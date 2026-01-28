"""
Lider Seçimi (Leader Selection) with PPO - ENTEGRE VERSİYON
============================================================

Bu modül, lider_sec.py simülasyonuna entegre PPO lider seçimi yapar.
- Actor: Lider seçim politikası (stochastic)
- Critic: State value estimation
- Action: Hangi ROV lider olsun (0-5 discrete actions)
- Reward: Mission success + battery efficiency + time efficiency

KULLANIM:
    python lider_sec_ppo.py  # Simülasyon + PPO eğitimi birlikte çalışır
"""
import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

# Working directory'yi REPO_ROOT'a değiştir (Ursina asset loading için)
os.chdir(REPO_ROOT)

from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.distributions import Categorical
from collections import deque
import math
from typing import List, Tuple, Dict, Optional


# =============================================================================
# ACTOR-CRITIC NETWORK
# =============================================================================
class ActorCriticNetwork(nn.Module):
    """
    Actor-Critic Network for PPO leader selection
    Actor: Policy (leader selection probabilities)
    Critic: Value function (state value)
    """
    def __init__(self, state_size=30, action_size=6, hidden_size=128):
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
    
    def get_batches(self, batch_size=32):
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
# PPO LİDER SEÇİM AGENT
# =============================================================================
class LiderSecimPPOAgent:
    """
    PPO tabanlı lider seçim agent'ı
    Simülasyon ile entegre çalışır
    """
    
    def __init__(self, num_rovs: int = 6, learning_rate: float = 0.0003,
                 gamma: float = 0.99, gae_lambda: float = 0.95, 
                 epsilon_clip: float = 0.2, epochs: int = 10):
        """
        Args:
            num_rovs: ROV sayısı
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            gae_lambda: GAE lambda parameter
            epsilon_clip: PPO clipping parameter
            epochs: PPO update epochs
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.num_rovs = num_rovs
        
        # Actor-Critic Network
        state_size = num_rovs * 5  # Her ROV: batarya, x, y, z, hedef_mesafesi
        self.policy = ActorCriticNetwork(state_size=state_size, action_size=num_rovs, 
                                        hidden_size=128).to(self.device)
        
        # Optimizer
        self.optimizer = optim.Adam(self.policy.parameters(), lr=learning_rate)
        
        # Hiperparametreler
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.epsilon_clip = epsilon_clip
        self.epochs = epochs
        
        # Memory
        self.memory = PPOMemory()
        
        print(f"🔧 [LiderSecPPO] Device: {self.device}")
    
    def extract_state(self, rovs_info: List[Dict]) -> np.ndarray:
        """State vektörü oluştur"""
        state_list = []
        
        for rov_info in rovs_info:
            # Batarya (normalize 0-1)
            state_list.append(rov_info['batarya'] / 100.0)
            
            # Konum (normalize)
            x, y, z = rov_info['konum']
            state_list.append(x / 500.0)
            state_list.append(y / 500.0)
            state_list.append(z / 500.0)
            
            # Hedef mesafesi (normalize)
            state_list.append(rov_info['hedef_mesafesi'] / 500.0)
        
        state = np.array(state_list, dtype=np.float32)
        
        # Padding
        state_size = self.num_rovs * 5
        if len(state) < state_size:
            state = np.pad(state, (0, state_size - len(state)), mode='constant')
        
        return state[:state_size]
    
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
    
    def calculate_reward(self, leader_id: int, mission_success: bool,
                        battery_level: float, time_efficiency: float) -> float:
        """Reward hesaplama"""
        mission_bonus = 100.0 if mission_success else -50.0
        battery_reward = (battery_level / 100.0) * 30.0
        efficiency_reward = time_efficiency * 20.0
        
        return mission_bonus + battery_reward + efficiency_reward
    
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
            for batch in self.memory.get_batches(batch_size=32):
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
    
    def select_leader_with_ppo(self, rovs_info: List[Dict], training: bool = False) -> int:
        """PPO ile lider seç"""
        state = self.extract_state(rovs_info)
        leader_id, _, _ = self.select_action(state, training=training)
        return leader_id
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'policy': self.policy.state_dict(),
            'optimizer': self.optimizer.state_dict()
        }, filepath)
        print(f"✅ [LiderSecPPO] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.policy.load_state_dict(checkpoint['policy'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        print(f"✅ [LiderSecPPO] Model yüklendi: {filepath}")


# =============================================================================
# ORİJİNAL LİDER SEÇİM ALGORITMASI (Yedek)
# =============================================================================
class LiderSecimModulu:
    """Orijinal formül tabanlı lider seçimi"""
    
    def __init__(self):
        pass

    def mesafe_hesapla(self, pos1, pos2):
        """İki nokta arası Öklid mesafesi"""
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def a_star_simulasyonu(self, baslangic, hedef):
        """Simülasyon amaçlı A* mesafesi (Kuş uçuşu * 1.2)"""
        kus_bakisi = self.mesafe_hesapla(baslangic, hedef)
        return kus_bakisi * 1.2 

    def deger_duzenle(self, deger):
        """KURAL: Değer 1'den küçükse 1'e yuvarla"""
        if deger < 1:
            return 1.0
        return deger

    def lideri_belirle_ve_yazdir(self, rov_listesi, hedef_konum):
        lider_skorlari = []
        
        # Merkez hesabı
        merkez_uzakliklari = []
        for i in range(len(rov_listesi)):
            toplam_mesafe = 0
            for j in range(len(rov_listesi)):
                if i == j: continue 
                dist = self.mesafe_hesapla(rov_listesi[i]['konum'], rov_listesi[j]['konum'])
                toplam_mesafe += dist
            merkez_uzakliklari.append(toplam_mesafe)

        # Hesaplama döngüsü
        for i, rov in enumerate(rov_listesi):
            p1 = rov['batarya'] / 100.0
            ham_derinlik = abs(rov['konum'][2]) 
            p2 = self.deger_duzenle(ham_derinlik)
            ham_mesafe = self.a_star_simulasyonu(rov['konum'], hedef_konum)
            p3 = self.deger_duzenle(ham_mesafe)
            ham_merkez = merkez_uzakliklari[i]
            p4 = self.deger_duzenle(ham_merkez)
            
            payda = p2 * p3 * p4
            skor = p1 / payda
            lider_skorlari.append(skor)

        if not lider_skorlari:
            return -1, 0

        max_skor = max(lider_skorlari)
        lider_index = lider_skorlari.index(max_skor)
        secilen_rov_id = rov_listesi[lider_index]['id']
        
        return secilen_rov_id, max_skor


# =============================================================================
# ENTEGRE LİDER SEÇİM FONKSİYONU
# =============================================================================
def liderlik_secimini_baslat_ppo(filo_nesnesi, hedef_konum, ppo_agent: Optional[LiderSecimPPOAgent] = None,
                                 use_ppo: bool = True, training: bool = False):
    """
    PPO veya orijinal algoritma ile lider seçimi
    
    Args:
        filo_nesnesi: Filo nesnesi
        hedef_konum: Hedef konum [x, y, z]
        ppo_agent: PPO agent (None ise orijinal algoritma kullanılır)
        use_ppo: PPO kullanılsın mı?
        training: Eğitim modu mu?
        
    Returns:
        (secilen_id, skor/reward)
    """
    # Veri toplama
    rovlar_listesi = []
    
    try:
        sistem_sayisi = len(filo_nesnesi.sistemler)
        
        for rid in range(sistem_sayisi):
            bat = filo_nesnesi.get(rid, "batarya") * 100 
            gps = filo_nesnesi.get(rid, "gps")
            
            # Hedef mesafesi
            hedef_mesafe = math.sqrt(
                (gps[0] - hedef_konum[0])**2 +
                (gps[1] - hedef_konum[1])**2 +
                (gps[2] - hedef_konum[2])**2
            )
            
            rovlar_listesi.append({
                'id': rid,
                'batarya': bat,
                'konum': gps,
                'hedef_mesafesi': hedef_mesafe,
                'merkezilik': 0.0
            })
            
    except Exception as e:
        print(f"Veri çekme hatası: {e}")
        return 0, 0
    
    # Merkezilik hesapla
    for i in range(len(rovlar_listesi)):
        toplam_mesafe = 0
        for j in range(len(rovlar_listesi)):
            if i == j: continue
            dist = math.sqrt(
                (rovlar_listesi[i]['konum'][0] - rovlar_listesi[j]['konum'][0])**2 +
                (rovlar_listesi[i]['konum'][1] - rovlar_listesi[j]['konum'][1])**2
            )
            toplam_mesafe += dist
        rovlar_listesi[i]['merkezilik'] = toplam_mesafe
    
    # Lider seçimi
    if use_ppo and ppo_agent is not None:
        # PPO ile seçim
        secilen_id = ppo_agent.select_leader_with_ppo(rovlar_listesi, training=training)
        skor = 0.0
    else:
        # Orijinal algoritma
        lider_modulu = LiderSecimModulu()
        secilen_id, skor = lider_modulu.lideri_belirle_ve_yazdir(rovlar_listesi, hedef_konum)
    
    return secilen_id, skor


# =============================================================================
# SİMÜLASYON ENTEGRASYONU
# =============================================================================

# Global değişkenler
app = None
filo = None
beyin = None
ppo_agent = None
training_mode = True
episode_count = 0
episode_rewards = []
episode_steps = 0
last_leader = None

def takipci_yap(lider_olacak):
    """Lider hariç diğerlerini takipçi yap"""
    for i in range(len(filo.sistemler)):
        if i != lider_olacak:
            filo.set(i, "rol", 0)
            x, y, z = filo.get(i, "gps")
            filo.git(i, x, y, -10)

def lider_kim():
    """Mevcut lideri bul"""
    for i in range(len(filo.sistemler)):
        rol = filo.get(i, "rol")
        if rol == 1:
            return i
    return 0

def update():
    """Ana güncelleme döngüsü (PPO entegreli)"""
    global episode_count, episode_steps, last_leader, episode_rewards
    
    try:
        # Lider seçimi (PPO veya orijinal)
        lider_id, skor = liderlik_secimini_baslat_ppo(
            filo, 
            filo.asil_hedef,
            ppo_agent=ppo_agent,
            use_ppo=training_mode,
            training=training_mode
        )
        onceki_lider = lider_kim()
        
        # Lider değişti mi?
        if lider_id != onceki_lider:
            filo.set(lider_id, "rol", 1)
            takipci_yap(lider_id)
            
            # PPO eğitimi için reward hesapla
            if training_mode and ppo_agent is not None and last_leader is not None:
                # Önceki liderin performansına göre reward
                battery_level = filo.get(last_leader, "batarya") * 100
                time_efficiency = 1.0 / max(episode_steps, 1)
                
                # Basit mission success kontrolü
                gps = filo.get(last_leader, "gps")
                dist_to_goal = math.sqrt(
                    (gps[0] - filo.asil_hedef[0])**2 +
                    (gps[1] - filo.asil_hedef[1])**2
                )
                mission_success = dist_to_goal < 20.0
                
                reward = ppo_agent.calculate_reward(
                    last_leader, mission_success, battery_level, time_efficiency
                )
                
                episode_rewards.append(reward)
                
                # PPO update
                if len(ppo_agent.memory.states) > 0:
                    policy_loss, value_loss, entropy = ppo_agent.ppo_update()
                    ppo_agent.memory.clear()
            
            last_leader = lider_id
            episode_steps = 0
            episode_count += 1
            
            # İstatistikler
            if episode_count % 50 == 0 and len(episode_rewards) > 0:
                avg_reward = np.mean(episode_rewards[-50:])
                print(f"📈 Episode {episode_count} | Avg Reward: {avg_reward:.2f}")
        
        episode_steps += 1
        
        # Komut kuyruğunu işle
        filo.execute_queued_commands()
        
        # GAT analizi
        veri = app.simden_veriye()
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        if ai_aktif and beyin:
            try: 
                tahminler, _, _ = beyin.analiz_et(veri)
            except: 
                tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)

        # Görselleştirme
        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
        
        for i, gat_kodu in enumerate(tahminler):
            app.rovs[i].gat_kodu = gat_kodu
            
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            app.rovs[i].label.scale = 6000
            app.rovs[i].label.y = 300
            app.rovs[i].label.color = app.rovs[i].color 
            app.rovs[i].label.background = False
            
            gat_kodu = app.rovs[i].gat_kodu
            if 0 <= gat_kodu < len(durum_txts):
                app.rovs[i].label.text = durum_txts[gat_kodu] + str(i)
            else:
                app.rovs[i].label.text = f"GAT:{gat_kodu}+{str(i)}"
        
        filo.guncelle_hepsi(tahminler)
        
        # Harita güncelle
        if hasattr(app, 'harita') and app.harita is not None:
            try:
                app.harita.update()
            except:
                pass
        
    except Exception as e: 
        pass


# =============================================================================
# MAIN EXECUTION
# =============================================================================
if __name__ == "__main__":
    print("\n" + "="*80)
    print("🤖 LİDER SEÇİM PPO - SİMÜLASYON ENTEGRASYONU")
    print("="*80 + "\n")
    
    # Simülasyonu başlat
    app = Ortam()
    app.sim_olustur(6, 25)
    rovs = app.rovs
    filo = Filo()
    modem = filo.otomatik_kurulum(rovs, 3)
    app.filo = filo
    
    # GAT modeli yükle
    try: 
        model_yolu = os.path.join(REPO_ROOT, "rov_modeli_multi.pth")
        beyin = FiratAnalizci(model_yolu=model_yolu)
    except: 
        print("⚠️ Model yüklenemedi, AI devre dışı.")
        beyin = None
    
    # PPO Agent oluştur
    ppo_agent = LiderSecimPPOAgent(num_rovs=6, learning_rate=0.0003, 
                                    gamma=0.99, gae_lambda=0.95, epsilon_clip=0.2, epochs=10)
    
    # Model yüklemeyi dene
    model_path = "lider_secim_ppo_model.pth"
    if os.path.exists(model_path):
        try:
            ppo_agent.load_model(model_path)
            print("✅ PPO model yüklendi!")
        except:
            print("⚠️ PPO model yüklenemedi, sıfırdan başlanıyor")
    
    print(f"🎯 Training Mode: {training_mode}")
    print(f"🎯 PPO Agent Device: {ppo_agent.device}")
    print("="*80 + "\n")
    
    app.konsola_ekle("filo", filo)
    app.konsola_ekle("ppo_agent", ppo_agent)
    
    # Update fonksiyonunu ayarla
    app.set_update_function(update)
    
    # Simülasyonu çalıştır
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        # Model kaydet
        if ppo_agent and training_mode:
            ppo_agent.save_model(model_path)
            print(f"\n✅ PPO Model kaydedildi: {model_path}")
        pass
    finally: 
        os.system('stty sane')
        os._exit(0)
