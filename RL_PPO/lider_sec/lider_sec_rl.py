"""
Lider Seçimi (Leader Selection) with RL - ENTEGRE VERSİYON
===========================================================

Bu modül, lider_sec.py simülasyonuna entegre RL lider seçimi yapar.
- State: Her ROV'un batarya, konum, hedef mesafesi, merkezilik
- Action: Lider adayı seçimi (0-5 arası ROV ID)
- Reward: Başarılı görev tamamlama, enerji verimliliği, yol optimizasyonu

KULLANIM:
    python lider_sec_rl.py  # Simülasyon + RL eğitimi birlikte çalışır
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
from collections import deque
import math
from typing import List, Tuple, Dict, Optional


# =============================================================================
# RL NETWORK: Lider Seçim DQN
# =============================================================================
class LiderSecimDQN(nn.Module):
    """
    Deep Q-Network for Leader Selection
    State: ROV features (battery, position, distance to goal, centrality)
    Action: Which ROV to select as leader
    """
    def __init__(self, state_size=30, action_size=6, hidden_size=128):
        super(LiderSecimDQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc4 = nn.Linear(hidden_size // 2, action_size)
        
        self.dropout = nn.Dropout(0.2)
        self.layernorm1 = nn.LayerNorm(hidden_size)
        self.layernorm2 = nn.LayerNorm(hidden_size)
    
    def forward(self, state):
        """
        Args:
            state: (batch, state_size) - [rov0_features, rov1_features, ..., global_info]
        Returns:
            Q-values: (batch, action_size) - Q-values for each ROV as leader
        """
        x = F.relu(self.layernorm1(self.fc1(state)))
        x = self.dropout(x)
        x = F.relu(self.layernorm2(self.fc2(x)))
        x = self.dropout(x)
        x = F.relu(self.fc3(x))
        q_values = self.fc4(x)
        return q_values


# =============================================================================
# RL LIDER SEÇİM AGENT
# =============================================================================
class LiderSecimRLAgent:
    """
    RL tabanlı lider seçim agent'ı
    Simülasyon ile entegre çalışır
    """
    
    def __init__(self, num_rovs: int = 6, learning_rate: float = 0.001,
                 gamma: float = 0.99, epsilon: float = 0.1):
        """
        Args:
            num_rovs: ROV sayısı
            learning_rate: Öğrenme oranı
            gamma: Discount factor
            epsilon: Exploration rate
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.num_rovs = num_rovs
        
        # Networks
        state_size = num_rovs * 5  # Her ROV: batarya, x, y, z, görev_başarısı
        self.q_network = LiderSecRLNetwork(state_size=state_size, action_size=num_rovs).to(self.device)
        self.target_network = LiderSecRLNetwork(state_size=state_size, action_size=num_rovs).to(self.device)
        self.target_network.load_state_dict(self.q_network.state_dict())
        
        # Optimizer
        self.optimizer = torch.optim.Adam(self.q_network.parameters(), lr=learning_rate)
        self.criterion = nn.MSELoss()
        
        # Hiperparametreler
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = 0.995
        self.learning_rate = learning_rate
        
        # Memory
        self.memory = deque(maxlen=10000)
        self.batch_size = 32
        
        # Lider seçim kriteri
        self.criteria_weights = {
            'batarya': 1.0,
            'konum': 0.8,
            'hedef_mesafesi': 0.6,
            'merkezilik': 0.7
        }
    
    def extract_state(self, rovs_info: List[Dict]) -> np.ndarray:
        """
        State vektörünü oluştur
        
        Args:
            rovs_info: Her ROV hakkında:
                {
                    'id': int,
                    'batarya': float (0-100),
                    'konum': (x, y, z),
                    'hedef_mesafesi': float,
                    'merkezilik': float
                }
        
        Returns:
            State vektörü
        """
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
    
    def select_action(self, state: np.ndarray, training: bool = True) -> int:
        """Epsilon-greedy aksiyon seçimi"""
        if training and np.random.random() < self.epsilon:
            return np.random.randint(0, self.num_rovs)
        
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            q_values = self.q_network(state_tensor)
        
        return q_values.argmax(dim=1).item()
    
    def calculate_reward(self, leader_id: int, mission_success: bool,
                        battery_level: float, time_efficiency: float) -> float:
        """
        Reward hesaplama
        
        Args:
            leader_id: Seçilen lider ID
            mission_success: Görev başarılı mı?
            battery_level: Lider batarya seviyesi (0-100)
            time_efficiency: Zaman verimliliği (0-1)
            
        Returns:
            Reward değeri
        """
        mission_bonus = 100.0 if mission_success else -50.0
        battery_reward = (battery_level / 100.0) * 30.0
        efficiency_reward = time_efficiency * 20.0
        
        return mission_bonus + battery_reward + efficiency_reward
    
    def remember(self, state: np.ndarray, action: int, reward: float,
                next_state: np.ndarray, done: bool):
        """Memory'ye ekle"""
        self.memory.append((state, action, reward, next_state, done))
    
    def train(self):
        """DQN eğitim"""
        if len(self.memory) < self.batch_size:
            return 0.0
        
        batch = [self.memory[i] for i in np.random.choice(len(self.memory), self.batch_size, replace=False)]
        
        states = torch.FloatTensor(np.array([exp[0] for exp in batch])).to(self.device)
        actions = torch.LongTensor(np.array([exp[1] for exp in batch])).to(self.device)
        rewards = torch.FloatTensor(np.array([exp[2] for exp in batch])).to(self.device)
        next_states = torch.FloatTensor(np.array([exp[3] for exp in batch])).to(self.device)
        dones = torch.FloatTensor(np.array([exp[4] for exp in batch])).to(self.device)
        
        # Q-Learning
        q_values = self.q_network(states).gather(1, actions.unsqueeze(1)).squeeze(1)
        next_q_values = self.target_network(next_states).max(dim=1)[0]
        target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
        
        loss = self.criterion(q_values, target_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        self.epsilon *= self.epsilon_decay
        
        return loss.item()
    
    def update_target_network(self):
        """Target network'ü güncelle"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def select_leader_with_rl(self, rovs_info: List[Dict], training: bool = False) -> int:
        """
        RL kullanarak lider seç
        
        Args:
            rovs_info: ROV bilgileri
            training: Eğitim modu mu?
            
        Returns:
            Seçilen lider ROV ID
        """
        state = self.extract_state(rovs_info)
        leader_id = self.select_action(state, training=training)
        return leader_id
    
    def save_model(self, filepath: str):
        """Model'i kaydet"""
        torch.save({
            'q_network': self.q_network.state_dict(),
            'target_network': self.target_network.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon
        }, filepath)
        print(f"✅ [LiderSecRL] Model kaydedildi: {filepath}")
    
    def load_model(self, filepath: str):
        """Model'i yükle"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.q_network.load_state_dict(checkpoint['q_network'])
        self.target_network.load_state_dict(checkpoint['target_network'])
        self.optimizer.load_state_dict(checkpoint['optimizer'])
        self.epsilon = checkpoint['epsilon']
        print(f"✅ [LiderSecRL] Model yüklendi: {filepath}")


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
def liderlik_secimini_baslat_rl(filo_nesnesi, hedef_konum, rl_agent: Optional[LiderSecimRLAgent] = None,
                                use_rl: bool = True, training: bool = False):
    """
    RL veya orijinal algoritma ile lider seçimi
    
    Args:
        filo_nesnesi: Filo nesnesi
        hedef_konum: Hedef konum [x, y, z]
        rl_agent: RL agent (None ise orijinal algoritma kullanılır)
        use_rl: RL kullanılsın mı?
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
                'merkezilik': 0.0  # Placeholder
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
    if use_rl and rl_agent is not None:
        # RL ile seçim
        secilen_id = rl_agent.select_leader_with_rl(rovlar_listesi, training=training)
        skor = 0.0  # RL için skor yok, reward var
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
rl_agent = None
training_mode = True
episode_count = 0
episode_rewards = []
episode_steps = 0
last_leader = None
mission_start_battery = {}

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
    """Ana güncelleme döngüsü (RL entegreli)"""
    global episode_count, episode_steps, last_leader
    
    try:
        # Lider seçimi (RL veya orijinal)
        lider_id, skor = liderlik_secimini_baslat_rl(
            filo, 
            filo.asil_hedef,
            rl_agent=rl_agent,
            use_rl=training_mode,
            training=training_mode
        )
        onceki_lider = lider_kim()
        
        # Lider değişti mi?
        if lider_id != onceki_lider:
            filo.set(lider_id, "rol", 1)
            takipci_yap(lider_id)
            
            # RL eğitimi için reward hesapla
            if training_mode and rl_agent is not None and last_leader is not None:
                # Önceki liderin performansına göre reward
                battery_level = filo.get(last_leader, "batarya") * 100
                time_efficiency = 1.0 / max(episode_steps, 1)
                
                # Basit mission success kontrolü (hedef mesafesi)
                gps = filo.get(last_leader, "gps")
                dist_to_goal = math.sqrt(
                    (gps[0] - filo.asil_hedef[0])**2 +
                    (gps[1] - filo.asil_hedef[1])**2
                )
                mission_success = dist_to_goal < 20.0
                
                reward = rl_agent.calculate_reward(
                    last_leader, mission_success, battery_level, time_efficiency
                )
                
                episode_rewards.append(reward)
                
                # Train RL agent
                if len(rl_agent.memory) > 0:
                    loss = rl_agent.train()
                    if episode_count % 10 == 0:
                        rl_agent.update_target_network()
            
            last_leader = lider_id
            episode_steps = 0
            episode_count += 1
            
            # İstatistikler
            if episode_count % 50 == 0 and len(episode_rewards) > 0:
                avg_reward = np.mean(episode_rewards[-50:])
                print(f"📈 Episode {episode_count} | Avg Reward: {avg_reward:.2f} | Epsilon: {rl_agent.epsilon:.3f}")
        
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
    print("🤖 LİDER SEÇİM RL - SİMÜLASYON ENTEGRASYONu")
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
    
    # RL Agent oluştur
    rl_agent = LiderSecimRLAgent(num_rovs=6, learning_rate=0.001, gamma=0.99, epsilon=0.3)
    
    # Model yüklemeyi dene
    model_path = "lider_secim_rl_model.pth"
    if os.path.exists(model_path):
        try:
            rl_agent.load_model(model_path)
            print("✅ RL model yüklendi!")
        except:
            print("⚠️ RL model yüklenemedi, sıfırdan başlanıyor")
    
    print(f"🎯 Training Mode: {training_mode}")
    print(f"🎯 RL Agent Device: {rl_agent.device}")
    print("="*80 + "\n")
    
    app.konsola_ekle("filo", filo)
    app.konsola_ekle("rl_agent", rl_agent)
    
    # Update fonksiyonunu ayarla
    app.set_update_function(update)
    
    # Simülasyonu çalıştır
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        # Model kaydet
        if rl_agent and training_mode:
            rl_agent.save_model(model_path)
            print(f"\n✅ RL Model kaydedildi: {model_path}")
        pass
    finally: 
        os.system('stty sane')
        os._exit(0)
