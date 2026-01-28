"""
GAT Model Eğitimi with PPO (Proximal Policy Optimization)
=========================================================

Bu modül, GAT modelini PPO algoritması ile eğitir.
GAT çıktısı state olarak kullanılır, PPO ajanı bu state'e göre action seçer.

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
from torch.distributions import Categorical
from torch_geometric.nn import GATConv
from torch_geometric.data import Data
import numpy as np

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
        """GAT embedding'i al (PPO state için)"""
        x = self.conv1(x, edge_index)
        x = F.elu(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        return x  # Hidden representation


# =============================================================================
# PPO ACTOR-CRITIC NETWORK: GAT Embedding → Policy & Value
# =============================================================================
class GAT_ActorCritic(nn.Module):
    """
    Actor-Critic Network for PPO
    GAT embedding'i kullanarak policy ve value üretir
    """
    def __init__(self, embedding_size, action_size=6, hidden_size=128):
        super(GAT_ActorCritic, self).__init__()
        
        # Shared layers
        self.fc_shared = nn.Linear(embedding_size, hidden_size)
        
        # Actor head (policy)
        self.actor_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.actor_out = nn.Linear(hidden_size // 2, action_size)
        
        # Critic head (value)
        self.critic_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.critic_out = nn.Linear(hidden_size // 2, 1)
        
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, x):
        """
        Args:
            x: GAT embedding (node embeddings averaged)
        Returns:
            action_probs: Action probabilities
            state_value: State value
        """
        # Shared features
        shared = F.relu(self.fc_shared(x))
        shared = self.dropout(shared)
        
        # Actor (policy)
        actor_x = F.relu(self.actor_fc(shared))
        action_logits = self.actor_out(actor_x)
        action_probs = F.softmax(action_logits, dim=-1)
        
        # Critic (value)
        critic_x = F.relu(self.critic_fc(shared))
        state_value = self.critic_out(critic_x)
        
        return action_probs, state_value
    
    def get_action(self, x, deterministic=False):
        """
        Stochastic action sampling
        
        Args:
            x: State embedding
            deterministic: If True, use argmax; else sample
        Returns:
            action, log_prob, value
        """
        action_probs, state_value = self.forward(x)
        dist = Categorical(action_probs)
        
        if deterministic:
            action = torch.argmax(action_probs, dim=-1)
        else:
            action = dist.sample()
        
        log_prob = dist.log_prob(action)
        
        return action, log_prob, state_value
    
    def evaluate_actions(self, x, actions):
        """
        Evaluate actions for PPO update
        
        Args:
            x: State embeddings
            actions: Taken actions
        Returns:
            log_probs, state_values, entropy
        """
        action_probs, state_values = self.forward(x)
        dist = Categorical(action_probs)
        
        log_probs = dist.log_prob(actions)
        entropy = dist.entropy()
        
        return log_probs, state_values.squeeze(-1), entropy


# =============================================================================
# PPO MEMORY BUFFER
# =============================================================================
class PPOMemory:
    """PPO için trajectory buffer"""
    def __init__(self):
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
    
    def add(self, state, action, log_prob, reward, value, done):
        self.states.append(state)
        self.actions.append(action)
        self.log_probs.append(log_prob)
        self.rewards.append(reward)
        self.values.append(value)
        self.dones.append(done)
    
    def clear(self):
        self.states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.values = []
        self.dones = []
    
    def get(self):
        return {
            'states': np.array(self.states, dtype=np.float32),
            'actions': np.array(self.actions, dtype=np.int64),
            'log_probs': np.array(self.log_probs, dtype=np.float32),
            'rewards': np.array(self.rewards, dtype=np.float32),
            'values': np.array(self.values, dtype=np.float32),
            'dones': np.array(self.dones, dtype=np.float32)
        }
    
    def __len__(self):
        return len(self.states)


# =============================================================================
# PPO AGENT: GAT + PPO
# =============================================================================
class GAT_PPO_Agent:
    """
    GAT tabanlı PPO Agent
    GAT model ile state representation, PPO ile action selection
    """
    def __init__(self, gat_hidden=16, gat_heads=4, gat_dropout=0.1,
                 ppo_hidden=128, learning_rate=3e-4, gamma=0.99, gae_lambda=0.95,
                 clip_epsilon=0.2, value_coef=0.5, entropy_coef=0.01):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"🔧 [GAT-PPO] Device: {self.device}")
        
        # GAT Model (state encoder)
        self.gat = GAT_Modeli(hidden_channels=gat_hidden, num_heads=gat_heads, dropout=gat_dropout).to(self.device)
        
        # PPO Actor-Critic Network
        embedding_size = gat_hidden * gat_heads  # GAT output dimension
        self.policy = GAT_ActorCritic(embedding_size=embedding_size, action_size=6, hidden_size=ppo_hidden).to(self.device)
        
        # Optimizer
        self.optimizer = optim.Adam(list(self.gat.parameters()) + list(self.policy.parameters()), lr=learning_rate)
        
        # Hiperparametreler
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        
        # PPO Memory
        self.memory = PPOMemory()
        
        # PPO training parameters
        self.ppo_epochs = 4
        self.batch_size = 64
        self.max_grad_norm = 0.5
        
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
        PPO policy kullanarak aksiyon seç
        
        Args:
            data: Graph data
            training: Training mode (stochastic) or eval mode (deterministic)
        Returns:
            action, log_prob, value
        """
        state_embedding = self.get_state_embedding(data).unsqueeze(0)
        
        with torch.no_grad():
            action, log_prob, value = self.policy.get_action(state_embedding, deterministic=not training)
        
        return action.item(), log_prob.item(), value.item()
    
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
        predicted_matches = (true_labels == predicted_action).sum().item()
        accuracy = predicted_matches / n_nodes
        reward += accuracy * 10.0  # Doğruluk bazlı reward
        
        # 2. Tehlike tespiti bonus
        dangerous_labels = [1, 2, 3, 4]  # ENGEL, CARPISMA, KOPUK, UZAK
        if predicted_action in dangerous_labels:
            if (true_labels == predicted_action).any():
                reward += 5.0
        
        # 3. Yanlış tehlike tespiti cezası
        safe_mask = (true_labels == 0)
        if predicted_action != 0 and safe_mask.any():
            reward -= 3.0  # False alarm penalty
        
        # 4. Kritik durumları kaçırma cezası
        if (true_labels == 2).any() and predicted_action != 2:
            reward -= 10.0
        
        # 5. Engel tespiti reward
        if predicted_action == 1 and (true_labels == 1).any():
            reward += 3.0
        
        return reward
    
    def compute_gae(self, rewards, values, dones, next_value):
        """
        Generalized Advantage Estimation (GAE) hesapla
        
        Args:
            rewards: Reward dizisi
            values: Value dizisi
            dones: Done flags
            next_value: Son state'in value'su
        Returns:
            advantages, returns
        """
        advantages = np.zeros_like(rewards)
        lastgaelam = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                nextnonterminal = 1.0 - dones[t]
                nextvalue = next_value
            else:
                nextnonterminal = 1.0 - dones[t]
                nextvalue = values[t + 1]
            
            delta = rewards[t] + self.gamma * nextvalue * nextnonterminal - values[t]
            advantages[t] = lastgaelam = delta + self.gamma * self.gae_lambda * nextnonterminal * lastgaelam
        
        returns = advantages + values
        return advantages, returns
    
    def train_step(self):
        """PPO training step"""
        if len(self.memory) == 0:
            return None, None, None
        
        # Memory'den veri al
        data = self.memory.get()
        states = data['states']
        actions = data['actions']
        old_log_probs = data['log_probs']
        rewards = data['rewards']
        values = data['values']
        dones = data['dones']
        
        # Son value'yu hesapla (bootstrap için)
        with torch.no_grad():
            last_state = torch.FloatTensor(states[-1]).unsqueeze(0).to(self.device)
            _, last_value = self.policy(last_state)
            last_value = last_value.item()
        
        # GAE hesapla
        advantages, returns = self.compute_gae(rewards, values, dones, last_value)
        
        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Tensörlere dönüştür
        states_tensor = torch.FloatTensor(states).to(self.device)
        actions_tensor = torch.LongTensor(actions).to(self.device)
        old_log_probs_tensor = torch.FloatTensor(old_log_probs).to(self.device)
        advantages_tensor = torch.FloatTensor(advantages).to(self.device)
        returns_tensor = torch.FloatTensor(returns).to(self.device)
        
        # PPO epochs
        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0
        num_updates = 0
        
        for _ in range(self.ppo_epochs):
            # Mini-batch indices
            indices = np.arange(len(states))
            np.random.shuffle(indices)
            
            for start in range(0, len(states), self.batch_size):
                end = start + self.batch_size
                batch_indices = indices[start:end]
                
                # Batch data
                batch_states = states_tensor[batch_indices]
                batch_actions = actions_tensor[batch_indices]
                batch_old_log_probs = old_log_probs_tensor[batch_indices]
                batch_advantages = advantages_tensor[batch_indices]
                batch_returns = returns_tensor[batch_indices]
                
                # Evaluate actions
                new_log_probs, state_values, entropy = self.policy.evaluate_actions(
                    batch_states, batch_actions
                )
                
                # Ratio for PPO
                ratio = torch.exp(new_log_probs - batch_old_log_probs)
                
                # Clipped surrogate objective
                surr1 = ratio * batch_advantages
                surr2 = torch.clamp(ratio, 1.0 - self.clip_epsilon, 1.0 + self.clip_epsilon) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss
                value_loss = F.mse_loss(state_values, batch_returns)
                
                # Entropy bonus
                entropy_loss = -entropy.mean()
                
                # Total loss
                loss = policy_loss + self.value_coef * value_loss + self.entropy_coef * entropy_loss
                
                # Optimize
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    list(self.gat.parameters()) + list(self.policy.parameters()), 
                    self.max_grad_norm
                )
                self.optimizer.step()
                
                total_policy_loss += policy_loss.item()
                total_value_loss += value_loss.item()
                total_entropy += entropy.mean().item()
                num_updates += 1
        
        # Memory'yi temizle
        self.memory.clear()
        
        avg_policy_loss = total_policy_loss / num_updates
        avg_value_loss = total_value_loss / num_updates
        avg_entropy = total_entropy / num_updates
        
        return avg_policy_loss, avg_value_loss, avg_entropy
    
    def save_model(self, filepath):
        """Model'i kaydet"""
        torch.save({
            'gat': self.gat.state_dict(),
            'policy': self.policy.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'training_stats': self.training_stats
        }, filepath)
        print(f"✅ [GAT-PPO] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath):
        """Model'i yükle"""
        if not os.path.exists(filepath):
            print(f"⚠️ [GAT-PPO] Model dosyası bulunamadı: {filepath}")
            return
        
        checkpoint = torch.load(filepath, map_location=self.device)
        self.gat.load_state_dict(checkpoint['gat'])
        self.policy.load_state_dict(checkpoint['policy'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.training_stats = checkpoint.get('training_stats', self.training_stats)
        print(f"✅ [GAT-PPO] Model yüklendi: {filepath}")


# =============================================================================
# EĞİTİM FONKSİYONU
# =============================================================================
def train_gat_ppo(n_episodes=1000, max_steps=100, save_interval=100):
    """
    GAT-PPO modelini eğit
    
    Args:
        n_episodes: Episode sayısı
        max_steps: Episode başına maksimum adım
        save_interval: Model kaydetme aralığı
    """
    print("🚀 [GAT-PPO] Eğitim başlıyor...")
    print(f"   Episodes: {n_episodes}")
    print(f"   Max Steps per Episode: {max_steps}")
    print()
    
    # Agent oluştur
    agent = GAT_PPO_Agent(
        gat_hidden=16,
        gat_heads=4,
        gat_dropout=0.1,
        ppo_hidden=128,
        learning_rate=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_epsilon=0.2,
        value_coef=0.5,
        entropy_coef=0.01
    )
    
    # Eğitim istatistikleri
    episode_rewards = []
    episode_policy_losses = []
    episode_value_losses = []
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
            
            # Action seç (PPO policy)
            action, log_prob, value = agent.select_action(data, training=True)
            
            # Reward hesapla
            reward = agent.calculate_reward(action, data.y, data)
            
            # Accuracy hesapla
            predicted_matches = (data.y == action).sum().item()
            accuracy = predicted_matches / len(data.y)
            episode_correct += predicted_matches
            episode_total += len(data.y)
            
            # Done condition
            done = (step == max_steps - 1)
            
            # State embedding'i al ve memory'ye ekle
            state_emb = agent.get_state_embedding(data).cpu().numpy()
            agent.memory.add(state_emb, action, log_prob, reward, value, done)
            
            episode_reward += reward
            
            if done:
                break
        
        # Episode sonunda PPO güncelleme
        policy_loss, value_loss, entropy = agent.train_step()
        
        # Episode sonu istatistikler
        episode_rewards.append(episode_reward)
        if policy_loss is not None:
            episode_policy_losses.append(policy_loss)
            episode_value_losses.append(value_loss)
        
        episode_accuracy = episode_correct / episode_total if episode_total > 0 else 0
        episode_accuracies.append(episode_accuracy)
        
        # İstatistikler
        agent.training_stats['episode'] = episode + 1
        agent.training_stats['total_reward'] = episode_reward
        agent.training_stats['correct_predictions'] = episode_correct
        
        # Log
        if (episode + 1) % 10 == 0:
            avg_reward = np.mean(episode_rewards[-10:])
            avg_accuracy = np.mean(episode_accuracies[-10:])
            avg_policy_loss = np.mean(episode_policy_losses[-10:]) if episode_policy_losses else 0
            avg_value_loss = np.mean(episode_value_losses[-10:]) if episode_value_losses else 0
            print(f"📊 Episode {episode+1}/{n_episodes} | "
                  f"Avg Reward: {avg_reward:.2f} | "
                  f"Accuracy: {avg_accuracy:.2%} | "
                  f"Policy Loss: {avg_policy_loss:.4f} | "
                  f"Value Loss: {avg_value_loss:.4f}")
        
        # Model kaydet
        if (episode + 1) % save_interval == 0:
            model_path = f"gat_ppo_model_ep{episode+1}.pth"
            agent.save_model(model_path)
    
    # Final model
    agent.save_model("gat_ppo_model_final.pth")
    print("\n✅ [GAT-PPO] Eğitim tamamlandı!")
    
    return agent


# =============================================================================
# MAIN
# =============================================================================
if __name__ == "__main__":
    print("🎯 GAT-PPO Eğitim Sistemi")
    print("=" * 60)
    print("GAT modeli Proximal Policy Optimization ile eğitiliyor...")
    print("Actor-Critic mimarisi + GAE + Clipped Objective")
    print("=" * 60)
    print()
    
    # Eğitimi başlat
    trained_agent = train_gat_ppo(n_episodes=1000, max_steps=100, save_interval=100)
    
    print("\n✅ Tüm işlemler tamamlandı!")
