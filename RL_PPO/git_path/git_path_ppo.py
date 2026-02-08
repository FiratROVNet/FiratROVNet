"""
Git Path with Proximal Policy Optimization (PPO)
================================================

Bu modül, git_path() fonksiyonunu PPO ile optimize eder.
A* yerine PPO agent yol planlama yapar.

- Actor: Hareket seçim politikası (stochastic)
- Critic: State value estimation
- Action: 8 yön hareketi (N, S, E, W, NE, NW, SE, SW)
- Reward: Hedefe yaklaşma, engelden kaçınma, yol optimizasyonu
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
import numpy as np
import math
from collections import deque
from typing import List, Tuple, Optional


# =============================================================================
# ACTOR-CRITIC NETWORK
# =============================================================================
class ActorCriticNetwork(nn.Module):
    """
    Actor-Critic Network for PPO path planning
    Actor: Policy (action probabilities)
    Critic: Value function (state value)
    """
    def __init__(self, state_size=20, action_size=8, hidden_size=128):
        super(ActorCriticNetwork, self).__init__()
        
        # Shared layers
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        
        # Actor head (policy)
        self.actor_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.actor_head = nn.Linear(hidden_size // 2, action_size)
        
        # Critic head (value)
        self.critic_fc = nn.Linear(hidden_size, hidden_size // 2)
        self.critic_head = nn.Linear(hidden_size // 2, 1)
        
        self.dropout = nn.Dropout(0.2)
        self.layernorm1 = nn.LayerNorm(hidden_size)
        self.layernorm2 = nn.LayerNorm(hidden_size)
    
    def forward(self, state):
        """
        Args:
            state: (batch, state_size)
        Returns:
            action_probs: (batch, action_size) - Softmax probabilities
            state_value: (batch, 1) - Value estimation
        """
        # Shared features
        x = F.relu(self.layernorm1(self.fc1(state)))
        x = self.dropout(x)
        x = F.relu(self.layernorm2(self.fc2(x)))
        x = self.dropout(x)
        
        # Actor (policy)
        actor_x = F.relu(self.actor_fc(x))
        action_logits = self.actor_head(actor_x)
        action_probs = F.softmax(action_logits, dim=-1)
        
        # Critic (value)
        critic_x = F.relu(self.critic_fc(x))
        state_value = self.critic_head(critic_x)
        
        return action_probs, state_value
    
    def get_action(self, state):
        """Stochastic action sampling"""
        action_probs, state_value = self.forward(state)
        dist = Categorical(action_probs)
        action = dist.sample()
        action_log_prob = dist.log_prob(action)
        
        return action.item(), action_log_prob, state_value
    
    def evaluate_actions(self, states, actions):
        """Evaluate actions for PPO update"""
        action_probs, state_values = self.forward(states)
        dist = Categorical(action_probs)
        action_log_probs = dist.log_prob(actions)
        dist_entropy = dist.entropy()
        
        return action_log_probs, state_values, dist_entropy


# =============================================================================
# PPO MEMORY BUFFER
# =============================================================================
class PPOMemory:
    """
    Memory buffer for PPO (trajectory based)
    Stores full episodes for on-policy learning
    """
    def __init__(self):
        self.states = []
        self.actions = []
        self.rewards = []
        self.log_probs = []
        self.values = []
        self.dones = []
    
    def store(self, state, action, reward, log_prob, value, done):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.log_probs.append(log_prob)
        self.values.append(value)
        self.dones.append(done)
    
    def clear(self):
        self.states.clear()
        self.actions.clear()
        self.rewards.clear()
        self.log_probs.clear()
        self.values.clear()
        self.dones.clear()
    
    def get_batches(self, batch_size=64):
        """Generate mini-batches"""
        n_states = len(self.states)
        indices = np.arange(n_states)
        np.random.shuffle(indices)
        
        for start_idx in range(0, n_states, batch_size):
            end_idx = min(start_idx + batch_size, n_states)
            batch_indices = indices[start_idx:end_idx]
            
            yield (
                np.array([self.states[i] for i in batch_indices]),
                np.array([self.actions[i] for i in batch_indices]),
                np.array([self.rewards[i] for i in batch_indices]),
                np.array([self.log_probs[i] for i in batch_indices]),
                np.array([self.values[i] for i in batch_indices]),
                np.array([self.dones[i] for i in batch_indices])
            )


# =============================================================================
# PPO PATH PLANNER AGENT
# =============================================================================
class PathPlannerPPO:
    """
    PPO tabanlı yol planlayıcı
    A* algoritmasının PPO versiyonu
    """
    
    # 8 yönlü hareket (grid based)
    ACTIONS = {
        0: (0, 1, 0),    # İleri (North)
        1: (0, -1, 0),   # Geri (South)
        2: (1, 0, 0),    # Sağ (East)
        3: (-1, 0, 0),   # Sol (West)
        4: (1, 1, 0),    # Sağ-İleri (NE)
        5: (-1, 1, 0),   # Sol-İleri (NW)
        6: (1, -1, 0),   # Sağ-Geri (SE)
        7: (-1, -1, 0)   # Sol-Geri (SW)
    }
    
    def __init__(self, grid_size=200, step_size=5.0, learning_rate=0.0003,
                 gamma=0.95, gae_lambda=0.95, epsilon_clip=0.2, epochs=10):
        """
        Args:
            grid_size: Grid boyutu (200x200 varsayılan)
            step_size: Her adımda hareket mesafesi (5.0m varsayılan)
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda parameter
            epsilon_clip: PPO clipping parameter
            epochs: PPO update epochs
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"🔧 [PathPlanner-PPO] Device: {self.device}")
        
        self.grid_size = grid_size
        self.step_size = step_size
        
        # Actor-Critic Network
        self.policy = ActorCriticNetwork(state_size=20, action_size=8, hidden_size=128).to(self.device)
        
        # Optimizer
        self.optimizer = optim.Adam(self.policy.parameters(), lr=learning_rate)
        
        # Hiperparametreler
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.epsilon_clip = epsilon_clip
        self.epochs = epochs
        
        # Memory
        self.memory = PPOMemory()
        
        # İstatistikler
        self.training_stats = {
            'episode': 0,
            'success_rate': 0,
            'avg_path_length': 0,
            'avg_reward': 0
        }
    
    def extract_state(self, current_pos: Tuple[float, float, float],
                     goal_pos: Tuple[float, float, float],
                     obstacles: List[Tuple[float, float, float]]) -> np.ndarray:
        """State vektörü oluştur (git_path_rl.py ile aynı)"""
        # Pozisyonlar (normalize edilmiş)
        state = [
            current_pos[0] / self.grid_size,  # 0: ROV X
            current_pos[1] / self.grid_size,  # 1: ROV Y
            current_pos[2] / 50.0,            # 2: ROV Z (depth)
            goal_pos[0] / self.grid_size,     # 3: Goal X
            goal_pos[1] / self.grid_size,     # 4: Goal Y
            goal_pos[2] / 50.0,               # 5: Goal Z
        ]
        
        # Hedefe mesafe ve açı
        dx = goal_pos[0] - current_pos[0]
        dy = goal_pos[1] - current_pos[1]
        dz = goal_pos[2] - current_pos[2]
        
        dist_to_goal = math.sqrt(dx**2 + dy**2 + dz**2)
        angle_to_goal = math.atan2(dy, dx)
        
        state.extend([
            dist_to_goal / self.grid_size,
            angle_to_goal / math.pi,
        ])
        
        # En yakın engeller (8 yön için)
        obstacle_distances = []
        for action_id in range(8):
            dx_act, dy_act, _ = self.ACTIONS[action_id]
            min_dist = self.grid_size
            
            if obstacles:
                for obs in obstacles:
                    obs_dx = obs[0] - current_pos[0]
                    obs_dy = obs[1] - current_pos[1]
                    dot_product = dx_act * obs_dx + dy_act * obs_dy
                    
                    if dot_product > 0:
                        dist = math.sqrt(obs_dx**2 + obs_dy**2)
                        min_dist = min(min_dist, dist)
            
            obstacle_distances.append(min_dist / self.grid_size)
        
        state.extend(obstacle_distances)
        
        # Grid sınırlarına mesafe
        border_distances = [
            (self.grid_size/2 - current_pos[1]) / self.grid_size,
            (self.grid_size/2 + current_pos[1]) / self.grid_size,
            (self.grid_size/2 - current_pos[0]) / self.grid_size,
            (self.grid_size/2 + current_pos[0]) / self.grid_size,
        ]
        state.extend(border_distances)
        
        return np.array(state, dtype=np.float32)
    
    def select_action(self, state: np.ndarray, training: bool = True):
        """PPO stochastic action selection"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        
        if training:
            action, log_prob, value = self.policy.get_action(state_tensor)
            return action, log_prob.item(), value.item()
        else:
            with torch.no_grad():
                action_probs, _ = self.policy(state_tensor)
                action = action_probs.argmax(dim=1).item()
            return action, 0, 0
    
    def calculate_reward(self, current_pos, next_pos, goal_pos, obstacles, done, collision):
        """Reward hesapla (git_path_rl.py ile aynı)"""
        current_dist = math.sqrt(
            (current_pos[0] - goal_pos[0])**2 + 
            (current_pos[1] - goal_pos[1])**2 + 
            (current_pos[2] - goal_pos[2])**2
        )
        
        next_dist = math.sqrt(
            (next_pos[0] - goal_pos[0])**2 + 
            (next_pos[1] - goal_pos[1])**2 + 
            (next_pos[2] - goal_pos[2])**2
        )
        
        reward = -0.1
        
        if next_dist < current_dist:
            reward += 1.0
        else:
            reward -= 0.5
        
        if collision:
            reward = -50.0
        elif done:
            reward = 100.0
        
        if obstacles:
            min_obstacle_dist = min([
                math.sqrt((obs[0]-next_pos[0])**2 + (obs[1]-next_pos[1])**2)
                for obs in obstacles
            ])
            if min_obstacle_dist < self.step_size * 2:
                reward -= 2.0
        
        return reward
    
    def compute_gae(self, rewards, values, dones):
        """Generalized Advantage Estimation"""
        advantages = []
        gae = 0
        
        for t in reversed(range(len(rewards))):
            if t == len(rewards) - 1:
                next_value = 0
            else:
                next_value = values[t + 1]
            
            delta = rewards[t] + self.gamma * next_value * (1 - dones[t]) - values[t]
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[t]) * gae
            advantages.insert(0, gae)
        
        returns = [adv + val for adv, val in zip(advantages, values)]
        
        return advantages, returns
    
    def ppo_update(self):
        """PPO policy update"""
        if len(self.memory.states) == 0:
            return 0, 0, 0
        
        # Compute GAE
        advantages, returns = self.compute_gae(
            self.memory.rewards,
            self.memory.values,
            self.memory.dones
        )
        
        # Normalize advantages
        advantages = np.array(advantages)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Convert to tensors
        old_states = torch.FloatTensor(np.array(self.memory.states)).to(self.device)
        old_actions = torch.LongTensor(self.memory.actions).to(self.device)
        old_log_probs = torch.FloatTensor(self.memory.log_probs).to(self.device)
        advantages_tensor = torch.FloatTensor(advantages).to(self.device)
        returns_tensor = torch.FloatTensor(returns).to(self.device)
        
        total_policy_loss = 0
        total_value_loss = 0
        total_entropy = 0
        update_count = 0
        
        # PPO epochs
        for _ in range(self.epochs):
            for batch in self.memory.get_batches(batch_size=64):
                batch_states, batch_actions, _, batch_old_log_probs, _, _ = batch
                
                batch_states_t = torch.FloatTensor(batch_states).to(self.device)
                batch_actions_t = torch.LongTensor(batch_actions).to(self.device)
                batch_old_log_probs_t = torch.FloatTensor(batch_old_log_probs).to(self.device)
                
                # Find indices in full data
                batch_indices = []
                for i in range(len(batch_states)):
                    for j in range(len(self.memory.states)):
                        if np.array_equal(batch_states[i], self.memory.states[j]):
                            batch_indices.append(j)
                            break
                
                batch_advantages = advantages_tensor[batch_indices]
                batch_returns = returns_tensor[batch_indices]
                
                # Evaluate actions
                new_log_probs, state_values, entropy = self.policy.evaluate_actions(
                    batch_states_t, batch_actions_t
                )
                
                # Policy loss (clipped surrogate objective)
                ratios = torch.exp(new_log_probs - batch_old_log_probs_t)
                surr1 = ratios * batch_advantages
                surr2 = torch.clamp(ratios, 1 - self.epsilon_clip, 1 + self.epsilon_clip) * batch_advantages
                policy_loss = -torch.min(surr1, surr2).mean()
                
                # Value loss
                value_loss = F.mse_loss(state_values.squeeze(), batch_returns)
                
                # Total loss
                loss = policy_loss + 0.5 * value_loss - 0.01 * entropy.mean()
                
                # Backprop
                self.optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(self.policy.parameters(), max_norm=0.5)
                self.optimizer.step()
                
                total_policy_loss += policy_loss.item()
                total_value_loss += value_loss.item()
                total_entropy += entropy.mean().item()
                update_count += 1
        
        avg_policy_loss = total_policy_loss / max(update_count, 1)
        avg_value_loss = total_value_loss / max(update_count, 1)
        avg_entropy = total_entropy / max(update_count, 1)
        
        return avg_policy_loss, avg_value_loss, avg_entropy
    
    def plan_path(self, start_pos: Tuple[float, float, float],
                 goal_pos: Tuple[float, float, float],
                 obstacles: List[Tuple[float, float, float]],
                 max_steps: int = 200) -> List[Tuple[float, float, float]]:
        """PPO ile yol planlama (inference)"""
        path = [start_pos]
        current_pos = list(start_pos)
        
        for step in range(max_steps):
            state = self.extract_state(tuple(current_pos), goal_pos, obstacles)
            action, _, _ = self.select_action(state, training=False)
            
            dx, dy, dz = self.ACTIONS[action]
            next_pos = [
                current_pos[0] + dx * self.step_size,
                current_pos[1] + dy * self.step_size,
                current_pos[2] + dz * self.step_size
            ]
            
            dist_to_goal = math.sqrt(
                (next_pos[0] - goal_pos[0])**2 + 
                (next_pos[1] - goal_pos[1])**2 + 
                (next_pos[2] - goal_pos[2])**2
            )
            
            if dist_to_goal < self.step_size:
                path.append(goal_pos)
                break
            
            path.append(tuple(next_pos))
            current_pos = next_pos
        
        return path
    
    def train(self, num_episodes: int = 1000, save_path: str = "path_planner_ppo.pth"):
        """
        PPO Agent'ı eğit
        
        Args:
            num_episodes: Episode sayısı
            save_path: Model kayıt yolu
        """
        print(f"\n{'='*80}")
        print(f"🚀 PATH PLANNER PPO TRAINING BAŞLIYOR")
        print(f"{'='*80}")
        print(f"📊 Episodes: {num_episodes}")
        print(f"📊 Device: {self.device}")
        print(f"📊 Grid Size: {self.grid_size}x{self.grid_size}")
        print(f"📊 Step Size: {self.step_size}m")
        print(f"📊 PPO Epochs: {self.epochs}")
        print(f"📊 Epsilon Clip: {self.epsilon_clip}")
        print(f"{'='*80}\n")
        
        episode_rewards = []
        episode_lengths = []
        success_count = 0
        
        for episode in range(num_episodes):
            # Random start ve goal pozisyonları
            start_pos = (
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(0, 30)
            )
            
            goal_pos = (
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(-self.grid_size/4, self.grid_size/4),
                np.random.uniform(0, 30)
            )
            
            # Random obstacles
            num_obstacles = np.random.randint(5, 15)
            obstacles = [
                (np.random.uniform(-self.grid_size/3, self.grid_size/3),
                 np.random.uniform(-self.grid_size/3, self.grid_size/3),
                 np.random.uniform(0, 30))
                for _ in range(num_obstacles)
            ]
            
            current_pos = list(start_pos)
            episode_reward = 0
            path_length = 0
            
            # Episode rollout
            for step in range(200):
                state = self.extract_state(tuple(current_pos), goal_pos, obstacles)
                action, log_prob, value = self.select_action(state, training=True)
                
                dx, dy, dz = self.ACTIONS[action]
                next_pos = [
                    current_pos[0] + dx * self.step_size,
                    current_pos[1] + dy * self.step_size,
                    current_pos[2] + dz * self.step_size
                ]
                
                # Check goal reach
                dist_to_goal = math.sqrt(
                    (next_pos[0] - goal_pos[0])**2 + 
                    (next_pos[1] - goal_pos[1])**2 + 
                    (next_pos[2] - goal_pos[2])**2
                )
                done = dist_to_goal < self.step_size
                
                # Check collision
                collision = False
                for obs in obstacles:
                    obs_dist = math.sqrt(
                        (next_pos[0] - obs[0])**2 + 
                        (next_pos[1] - obs[1])**2
                    )
                    if obs_dist < self.step_size:
                        collision = True
                        break
                
                # Check bounds
                if (abs(next_pos[0]) > self.grid_size/2 or 
                    abs(next_pos[1]) > self.grid_size/2 or 
                    next_pos[2] < 0 or next_pos[2] > 50):
                    collision = True
                
                reward = self.calculate_reward(current_pos, next_pos, goal_pos, 
                                               obstacles, done, collision)
                
                # Store transition
                self.memory.store(state, action, reward, log_prob, value, done or collision)
                
                episode_reward += reward
                path_length += 1
                
                if done:
                    success_count += 1
                    break
                
                if collision:
                    break
                
                current_pos = next_pos
            
            # PPO update after episode
            policy_loss, value_loss, entropy = self.ppo_update()
            self.memory.clear()
            
            episode_rewards.append(episode_reward)
            episode_lengths.append(path_length)
            
            # Logging
            if episode % 50 == 0:
                avg_reward = np.mean(episode_rewards[-50:]) if len(episode_rewards) >= 50 else np.mean(episode_rewards)
                avg_length = np.mean(episode_lengths[-50:]) if len(episode_lengths) >= 50 else np.mean(episode_lengths)
                success_rate = success_count / (episode + 1) * 100
                
                print(f"📈 Episode {episode}/{num_episodes} | "
                      f"Avg Reward: {avg_reward:.2f} | "
                      f"Avg Length: {avg_length:.1f} | "
                      f"Success: {success_rate:.1f}% | "
                      f"Policy Loss: {policy_loss:.4f} | "
                      f"Value Loss: {value_loss:.4f}")
                
                self.training_stats['episode'] = episode
                self.training_stats['avg_reward'] = float(avg_reward)
                self.training_stats['avg_path_length'] = float(avg_length)
                self.training_stats['success_rate'] = float(success_rate)
        
        # Model kaydet
        self.save_model(save_path)
        
        print(f"\n{'='*80}")
        print(f"✅ EĞİTİM TAMAMLANDI!")
        print(f"📊 Toplam Episode: {num_episodes}")
        print(f"📊 Başarı Oranı: {success_count/num_episodes*100:.1f}%")
        print(f"📊 Ortalama Reward: {np.mean(episode_rewards):.2f}")
        print(f"📊 Ortalama Yol Uzunluğu: {np.mean(episode_lengths):.1f}")
        print(f"{'='*80}\n")
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'policy': self.policy.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'training_stats': self.training_stats
        }, filepath)
        print(f"✅ [PathPlanner-PPO] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.policy.load_state_dict(checkpoint['policy'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.training_stats = checkpoint['training_stats']
        print(f"✅ [PathPlanner-PPO] Model yüklendi: {filepath}")


# =============================================================================
# MAIN TRAINING LOOP
# =============================================================================
if __name__ == "__main__":
    print("\n" + "="*80)
    print("🤖 GIT_PATH PPO TRAINING")
    print("="*80 + "\n")
    
    # Agent oluştur
    agent = PathPlannerPPO(
        grid_size=200,
        step_size=5.0,
        learning_rate=0.0003,
        gamma=0.95,
        gae_lambda=0.95,
        epsilon_clip=0.2,
        epochs=10
    )
    
    # Eğitim
    agent.train(
        num_episodes=1000,
        save_path="path_planner_ppo_model.pth"
    )
    
    # Test
    print("\n" + "="*80)
    print("🧪 TEST PHASE")
    print("="*80 + "\n")
    
    start = (0, 0, 10)
    goal = (80, 80, 10)
    obstacles = [(30, 30, 10), (50, 50, 10), (70, 40, 10)]
    
    path = agent.plan_path(start, goal, obstacles, max_steps=200)
    
    print(f"✅ Path planned: {len(path)} waypoints")
    print(f"   Start: {start}")
    print(f"   Goal: {goal}")
    print(f"   Path length: {len(path)}")
    
    print("\n" + "="*80)
    print("✅ TAMAMLANDI!")
    print("="*80 + "\n")
