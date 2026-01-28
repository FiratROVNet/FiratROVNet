"""
GAT Model Eğitimi with Reinforcement Learning (DQN)
===================================================

Bu modül, GAT modelini DQN algoritması ile eğitir.
GAT çıktısı state olarak kullanılır, RL ajanı bu state'e göre action seçer.

- State: GAT embedding + ROV sensör verileri
- Action: GAT sınıf tahminleri (0=OK, 1=ENGEL, 2=CARPISMA, 3=KOPUK, 4=UZAK)
- Reward: Doğru tehlike tespiti, güvenli navigasyon, enerji verimliliği
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch_geometric.nn import GATConv
from torch_geometric.data import Data
import numpy as np
from collections import deque
import random

from FiratROVNet.ortam import veri_uret
from FiratROVNet.config import GATLimitleri


# =============================================================================
# GAT MODEL (Orijinal yapı korundu)
# =============================================================================
class GAT_Modeli(torch.nn.Module):
    """
    GAT Modeli - Graph Attention Network
    Input: 7 özellik, Output: 6 sınıf
    """
    def __init__(self, hidden_channels=16, num_heads=4, dropout=0.1):
        super().__init__()
        self.conv1 = GATConv(in_channels=7, out_channels=hidden_channels, heads=num_heads, dropout=dropout)
        self.conv2 = GATConv(hidden_channels * num_heads, 6, heads=1, dropout=dropout)
        self.dropout = dropout

    def forward(self, x, edge_index, return_attention=False):
        x = self.conv1(x, edge_index)
        x = F.elu(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        
        if return_attention:
            x, (ei, alpha) = self.conv2(x, edge_index, return_attention_weights=True)
            return F.log_softmax(x, dim=1), ei, alpha
        else:
            x = self.conv2(x, edge_index)
            return F.log_softmax(x, dim=1)
    
    def get_embedding(self, x, edge_index):
        """GAT embedding'i al (RL state için)"""
        x = self.conv1(x, edge_index)
        x = F.elu(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        return x  # Hidden representation


# =============================================================================
# DQN NETWORK: GAT Embedding + Q-Values
# =============================================================================
class GAT_DQN(nn.Module):
    """
    Deep Q-Network for GAT-based RL
    GAT embedding'i kullanarak Q-values üretir
    """
    def __init__(self, embedding_size, action_size=6, hidden_size=128):
        super(GAT_DQN, self).__init__()
        self.fc1 = nn.Linear(embedding_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, x):
        """
        Args:
            x: GAT embedding (node embeddings averaged)
        Returns:
            Q-values for each action
        """
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        q_values = self.fc3(x)
        return q_values


# =============================================================================
# RL AGENT: GAT + DQN
# =============================================================================
class GAT_RL_Agent:
    """
    GAT tabanlı Reinforcement Learning Agent
    GAT model ile state representation, DQN ile action selection
    """
    def __init__(self, gat_hidden=16, gat_heads=4, gat_dropout=0.1,
                 dqn_hidden=128, learning_rate=0.001, gamma=0.99, epsilon=1.0):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"🔧 [GAT-RL] Device: {self.device}")
        
        # GAT Model (state encoder)
        self.gat = GAT_Modeli(hidden_channels=gat_hidden, num_heads=gat_heads, dropout=gat_dropout).to(self.device)
        
        # DQN Network (Q-value estimator)
        embedding_size = gat_hidden * gat_heads  # GAT output dimension
        self.q_network = GAT_DQN(embedding_size=embedding_size, action_size=6, hidden_size=dqn_hidden).to(self.device)
        self.target_network = GAT_DQN(embedding_size=embedding_size, action_size=6, hidden_size=dqn_hidden).to(self.device)
        self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Optimizer
        self.optimizer = optim.Adam(list(self.gat.parameters()) + list(self.q_network.parameters()), lr=learning_rate)
        self.criterion = nn.MSELoss()
        
        # Hiperparametreler
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = 0.995
        self.epsilon_min = 0.01
        
        # Experience Replay
        self.memory = deque(maxlen=10000)
        self.batch_size = 64
        
        # İstatistikler
        self.training_stats = {
            'episode': 0,
            'total_reward': 0,
            'correct_predictions': 0,
            'avg_loss': 0
        }
    
    def get_state_embedding(self, data):
        """
        GAT model ile state embedding'i al
        
        Args:
            data: torch_geometric.data.Data (x, edge_index)
        Returns:
            Global state embedding (averaged node embeddings)
        """
        with torch.no_grad():
            node_embeddings = self.gat.get_embedding(data.x, data.edge_index)  # (n_nodes, embedding_dim)
            # Global pooling (mean)
            state_embedding = torch.mean(node_embeddings, dim=0)  # (embedding_dim,)
        return state_embedding
    
    def select_action(self, data, training=True):
        """
        Epsilon-greedy action selection
        
        Args:
            data: Graph data
            training: Training mode (epsilon-greedy) or eval mode (greedy)
        Returns:
            action: Selected action (0-5)
        """
        # Epsilon-greedy exploration
        if training and random.random() < self.epsilon:
            return random.randint(0, 5)
        
        # Greedy exploitation
        state_embedding = self.get_state_embedding(data)
        with torch.no_grad():
            q_values = self.q_network(state_embedding.unsqueeze(0))
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, predicted_action, true_labels, data):
        """
        Reward fonksiyonu
        
        Args:
            predicted_action: Agent'in seçtiği aksiyon (GAT sınıfı)
            true_labels: Gerçek GAT etiketleri
            data: Graph data
        Returns:
            reward: Hesaplanan reward değeri
        """
        reward = 0.0
        n_nodes = len(true_labels)
        
        # 1. Doğru tahmin reward'u (global metrik)
        # Tüm node'ların çoğunluğunun predicted_action ile eşleşme oranı
        predicted_matches = (true_labels == predicted_action).sum().item()
        accuracy = predicted_matches / n_nodes
        reward += accuracy * 10.0  # Doğruluk bazlı reward
        
        # 2. Tehlike tespiti bonus
        # Tehlikeli durumları (1,2,3,4) doğru tespit etmek daha önemli
        dangerous_labels = [1, 2, 3, 4]  # ENGEL, CARPISMA, KOPUK, UZAK
        if predicted_action in dangerous_labels:
            # Tehlikeli durumları doğru tespit ederse bonus
            if (true_labels == predicted_action).any():
                reward += 5.0
        
        # 3. Yanlış tehlike tespiti cezası
        # Güvenli durumda (0=OK) tehlike sinyali vermek zararlı
        safe_mask = (true_labels == 0)
        if predicted_action != 0 and safe_mask.any():
            reward -= 3.0  # False alarm penalty
        
        # 4. Kritik durumları kaçırma cezası
        # CARPISMA (2) durumunu kaçırmak çok kötü
        if (true_labels == 2).any() and predicted_action != 2:
            reward -= 10.0
        
        # 5. Engel tespiti reward
        # ENGEL (1) durumunu doğru tespit etmek iyi
        if predicted_action == 1 and (true_labels == 1).any():
            reward += 3.0
        
        return reward
    
    def remember(self, state_data, action, reward, next_state_data, done):
        """Experience memory'ye ekle"""
        # State'leri embedding olarak sakla (memory verimliliği için)
        state_emb = self.get_state_embedding(state_data).cpu().numpy()
        next_state_emb = self.get_state_embedding(next_state_data).cpu().numpy()
        self.memory.append((state_emb, action, reward, next_state_emb, done))
    
    def train_step(self):
        """DQN training step (experience replay)"""
        if len(self.memory) < self.batch_size:
            return None
        
        # Random batch sample
        batch = random.sample(self.memory, self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        # Numpy arrays -> Tensors
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.LongTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)
        
        # Current Q-values
        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        
        # Target Q-values (Double DQN)
        with torch.no_grad():
            next_actions = self.q_network(next_states).argmax(dim=1)
            next_q_values = self.target_network(next_states).gather(1, next_actions.unsqueeze(1)).squeeze(1)
            target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
        
        # Loss ve backprop
        loss = self.criterion(current_q_values, target_q_values)
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(list(self.gat.parameters()) + list(self.q_network.parameters()), 1.0)
        self.optimizer.step()
        
        # Epsilon decay
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
        
        return loss.item()
    
    def update_target_network(self):
        """Target network'ü güncelle"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def save_model(self, filepath):
        """Model'i kaydet"""
        torch.save({
            'gat': self.gat.state_dict(),
            'q_network': self.q_network.state_dict(),
            'target_network': self.target_network.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon,
            'training_stats': self.training_stats
        }, filepath)
        print(f"✅ [GAT-RL] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath):
        """Model'i yükle"""
        if not os.path.exists(filepath):
            print(f"⚠️ [GAT-RL] Model dosyası bulunamadı: {filepath}")
            return
        
        checkpoint = torch.load(filepath, map_location=self.device)
        self.gat.load_state_dict(checkpoint['gat'])
        self.q_network.load_state_dict(checkpoint['q_network'])
        self.target_network.load_state_dict(checkpoint['target_network'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.epsilon = checkpoint['epsilon']
        self.training_stats = checkpoint.get('training_stats', self.training_stats)
        print(f"✅ [GAT-RL] Model yüklendi: {filepath}")


# =============================================================================
# EĞİTİM FONKSİYONU
# =============================================================================
def train_gat_rl(n_episodes=1000, max_steps=100, save_interval=100):
    """
    GAT-RL modelini eğit
    
    Args:
        n_episodes: Episode sayısı
        max_steps: Episode başına maksimum adım
        save_interval: Model kaydetme aralığı
    """
    print("🚀 [GAT-RL] Eğitim başlıyor...")
    print(f"   Episodes: {n_episodes}")
    print(f"   Max Steps per Episode: {max_steps}")
    print()
    
    # Agent oluştur
    agent = GAT_RL_Agent(
        gat_hidden=16,
        gat_heads=4,
        gat_dropout=0.1,
        dqn_hidden=128,
        learning_rate=0.001,
        gamma=0.99,
        epsilon=1.0
    )
    
    # Eğitim istatistikleri
    episode_rewards = []
    episode_losses = []
    episode_accuracies = []
    
    for episode in range(n_episodes):
        episode_reward = 0
        episode_correct = 0
        episode_total = 0
        
        for step in range(max_steps):
            # Veri üret (simülasyon step)
            data = veri_uret()
            data.x = data.x.to(agent.device)
            data.edge_index = data.edge_index.to(agent.device)
            data.y = data.y.to(agent.device)
            
            # Action seç (GAT sınıfı tahmin et)
            action = agent.select_action(data, training=True)
            
            # Reward hesapla
            reward = agent.calculate_reward(action, data.y, data)
            
            # Accuracy hesapla
            predicted_matches = (data.y == action).sum().item()
            accuracy = predicted_matches / len(data.y)
            episode_correct += predicted_matches
            episode_total += len(data.y)
            
            # Next state (yeni veri)
            next_data = veri_uret()
            next_data.x = next_data.x.to(agent.device)
            next_data.edge_index = next_data.edge_index.to(agent.device)
            next_data.y = next_data.y.to(agent.device)
            
            # Done condition (episode bitiş koşulu)
            done = (step == max_steps - 1)
            
            # Memory'ye ekle
            agent.remember(data, action, reward, next_data, done)
            
            # Train
            if step % 4 == 0:  # Her 4 adımda bir train
                loss = agent.train_step()
                if loss is not None:
                    episode_losses.append(loss)
            
            episode_reward += reward
            
            if done:
                break
        
        # Episode sonu
        episode_rewards.append(episode_reward)
        episode_accuracy = episode_correct / episode_total if episode_total > 0 else 0
        episode_accuracies.append(episode_accuracy)
        
        # Target network güncelle
        if (episode + 1) % 10 == 0:
            agent.update_target_network()
        
        # İstatistikler
        agent.training_stats['episode'] = episode + 1
        agent.training_stats['total_reward'] = episode_reward
        agent.training_stats['correct_predictions'] = episode_correct
        
        # Log
        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            avg_accuracy = np.mean(episode_accuracies[-10:])
            avg_loss = np.mean(episode_losses[-10:]) if episode_losses else 0
            print(f"📊 Episode {episode+1}/{n_episodes} | "
                  f"Avg Reward: {avg_reward:.2f} | "
                  f"Accuracy: {avg_accuracy:.2%} | "
                  f"Loss: {avg_loss:.4f} | "
                  f"Epsilon: {agent.epsilon:.3f}")
        
        # Model kaydet
        if (episode + 1) % save_interval == 0:
            model_path = f"gat_rl_model_ep{episode+1}.pth"
            agent.save_model(model_path)
    
    # Final model
    agent.save_model("gat_rl_model_final.pth")
    print("\n✅ [GAT-RL] Eğitim tamamlandı!")
    
    return agent


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print("🎯 GAT-RL Eğitim Sistemi")
    print("=" * 60)
    print("GAT modeli Reinforcement Learning ile eğitiliyor...")
    print("DQN algoritması kullanılıyor (Double DQN + Experience Replay)")
    print("=" * 60)
    print()
    
    # Eğitimi başlat
    trained_agent = train_gat_rl(n_episodes=1000, max_steps=100, save_interval=100)
    
    print("\n✅ Tüm işlemler tamamlandı!")
