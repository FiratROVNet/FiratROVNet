"""
FiratROVNet - RL & PPO Model Dokumentasyon
==========================================

Bu dokümanda, tüm RL ve PPO modelleri ve bunların hangi dosyalarda bulunduğu açıklanmıştır.

ÖZET
====

Her temel fonksiyon için 2 model oluşturulmuştur:
1. RL Model: Q-Learning/DQN tabanlı 
2. PPO Model: Proximal Policy Optimization tabanlı

"""

# ============================================================
# 1. A* YOLU BULMA - A* PATH FINDING
# ============================================================
# Dosyalar: a_star_rl.py, a_star_ppo.py

"""
A* Yol Bulma (A* Path Finding):
- Amaç: Başlangıçtan hedefe en uygun yolu bulmak
- State: Mevcut pozisyon, hedef, engel bilgisi
- Action: 8 yön hareketi (sağ, sol, ileri, geri, vs.)

a_star_rl.py - RL Versiyonu:
  - Sınıf: A_StarRL
  - Q-Network tabanlı
  - Epsilon-greedy aksiyon seçimi
  - Experience replay buffer
  - Fonksiyonlar:
    * a_star_with_rl(): RL ile optimized yol bulma
    * extract_state(): State vektörü oluşturma
    * calculate_reward(): Hedefe yaklaşma reward'ı
    * train(): DQN eğitim

a_star_ppo.py - PPO Versiyonu:
  - Sınıflar: A_StarPPOActor, A_StarPPOCritic
  - Actor-Critic mimarisi
  - Generalized Advantage Estimation (GAE)
  - PPO clipped loss
  - Fonksiyonlar:
    * a_star_with_ppo(): PPO ile optimized yol bulma
    * calculate_gae(): Advantage estimation
    * train(): PPO eğitim loop'u
"""

# ============================================================
# 2. FORMASYON SEÇIMI - FORMATION
# ============================================================
# Dosyalar: formasyon_rl_enhanced.py

"""
Formasyon Seçimi (Formation):
- Amaç: Filo'yu belirtilen şekilde düzenlemek
- State: ROV pozisyonları, lider pozisyonu, hedef
- Action: 20 farklı formasyon tipi (LINE, V_SHAPE, DIAMOND, vs.)

formasyon_rl_enhanced.py - RL Versiyonu:
  - Sınıf: FormasyonRL_Enhanced
  - Q-Network tabanlı formasyon seçimi
  - Formasyon tutarlılığı ve enerji verimliliği ödülleri
  - Fonksiyonlar:
    * select_formation_with_rl(): En uygun formasyonu seç
    * calculate_reward(): Formasyon kalitesi reward'ı
    * train(): DQN eğitim
"""

# ============================================================
# 3. FORMASYON SEÇİM OPTİMİZASYONU - FORMATION SELECTION
# ============================================================
# Dosyalar: formasyon_sec_rl.py, formasyon_sec_ppo.py

"""
Formasyon Seçim Optimizasyonu (Formation Selection):
- Amaç: Convex hull ile uyumlu en iyi formasyonu belirlemek
- State: ROV pozisyonları, hull bilgisi, hedef
- Action: 20 formasyon tipi + parametreleri

formasyon_sec_rl.py - RL Versiyonu:
  - Sınıf: FormasyonSecRL
  - Q-Network tabanlı
  - Hull fitnesss hesaplaması
  - Fonksiyonlar:
    * select_formation_with_hull_rl(): Hull bilgisi ile formasyon seç
    * calculate_reward(): Hull uygunluğu ve güvenlik reward'ı
    * train(): DQN eğitim

formasyon_sec_ppo.py - PPO Versiyonu:
  - Sınıflar: FormasyonPPOActor, FormasyonPPOCritic
  - Actor-Critic mimarisi
  - GAE ve PPO loss
  - Fonksiyonlar:
    * select_formation_with_ppo(): PPO ile formasyon seç
    * calculate_gae(): Advantage estimation
    * train(): PPO eğitim
"""

# ============================================================
# 4. LİDER SEÇİMİ - LEADER SELECTION
# ============================================================
# Dosyalar: lider_sec_rl.py, lider_sec_ppo.py

"""
Lider Seçimi (Leader Selection):
- Amaç: Filosundan en uygun lideri belirlemek
- State: Her ROV'un batarya, konum, hedef mesafesi, merkezilik
- Action: ROV ID seçimi (0-5 arası)

lider_sec_rl.py - RL Versiyonu:
  - Sınıf: LiderSecRL
  - Q-Network tabanlı lider seçimi
  - Batarya, mesafe, merkezilik göz önüne alarak
  - Fonksiyonlar:
    * select_leader_with_rl(): En iyi lideri seç
    * calculate_reward(): Görev başarısı ve batarya reward'ı
    * train(): DQN eğitim

lider_sec_ppo.py - PPO Versiyonu:
  - Sınıflar: LiderSecPPOActor, LiderSecPPOCritic
  - Actor-Critic mimarisi
  - Politika optimizasyonu
  - Fonksiyonlar:
    * select_leader_with_ppo(): PPO ile lider seç
    * calculate_gae(): GAE hesaplama
    * train(): PPO eğitim
"""

# ============================================================
# 5. CONVEX HULL OPTİMİZASYONU - CONVEX HULL
# ============================================================
# Dosyalar: convex_hull_rl.py, convex_hull_ppo.py

"""
Convex Hull Optimizasyonu (Convex Hull):
- Amaç: Güvenli işlem alanı oluşturmak için optimal hull parametrelerini seçmek
- State: Engel pozisyonları, ROV pozisyonları, alan bilgisi
- Action: 10 farklı hull parametre kombinasyonu

convex_hull_rl.py - RL Versiyonu:
  - Sınıf: ConvexHullRL
  - Q-Network tabanlı parametre seçimi
  - Hull geçerliliği, kapsama, güvenlik reward'ları
  - Fonksiyonlar:
    * select_hull_params_with_rl(): En iyi hull parametrelerini seç
    * calculate_reward(): Hull kalitesi reward'ı
    * train(): DQN eğitim

convex_hull_ppo.py - PPO Versiyonu:
  - Sınıflar: ConvexHullPPOActor, ConvexHullPPOCritic
  - Actor-Critic mimarisi
  - PPO optimizasyon
  - Fonksiyonlar:
    * select_hull_params_with_ppo(): PPO ile hull parametreleri seç
    * calculate_gae(): GAE hesaplama
    * train(): PPO eğitim
"""

# ============================================================
# 6. YOL TAKİP - GIT PATH
# ============================================================
# Dosyalar: git_path_rl.py, git_path_ppo.py

"""
Yol Takip (Git Path - Path Following):
- Amaç: Hesapladığı yolu adım adım takip etmek
- State: Mevcut pozisyon, hedef, yol bilgisi, batarya
- Action: 8 hareket yönü

git_path_rl.py - RL Versiyonu:
  - Sınıf: GitPathRL
  - Q-Network tabanlı hareket seçimi
  - Waypoint takip ve enerji verimliliği reward'ları
  - Fonksiyonlar:
    * get_movement_with_rl(): RL ile hareket belirle
    * calculate_reward(): Hedefe yaklaşma ve enerji reward'ı
    * train(): DQN eğitim

git_path_ppo.py - PPO Versiyonu:
  - Sınıflar: GitPathPPOActor, GitPathPPOCritic
  - Actor-Critic mimarisi
  - Politika optimizasyonu
  - Fonksiyonlar:
    * get_movement_with_ppo(): PPO ile hareket belirle
    * calculate_gae(): GAE hesaplama
    * train(): PPO eğitim
"""

# ============================================================
# TEKNIK DETAYLAR
# ============================================================

"""
RL (DQN) Modeller:
====================
- Q-Network: State -> Q-values (her action için)
- Target Network: Stability için (her 1000 adımda güncelleme)
- Experience Replay: Batch learning (10,000 transitions)
- Epsilon-Greedy: Exploration vs Exploitation
  * Başlangıç epsilon: 0.1
  * Decay: Her adımda * 0.995
  
PPO Modeller:
====================
- Actor Network: State -> Action probabilities (policy)
- Critic Network: State -> Value (expected return)
- Generalized Advantage Estimation (GAE):
  * Lambda: 0.95 (credit assignment)
  * Gamma: 0.99 (discount factor)
- PPO Clipping: 
  * Clip ratio: 0.2 (stability)
- Mini-batch training: 32 transitions
- Entropy bonus: Exploration (coef: 0.01)
"""

# ============================================================
# KULLANIM ÖRNEĞI
# ============================================================

"""
# A* Yol Bulma
from a_star_rl import A_StarRL
a_star_rl = A_StarRL()
path = a_star_rl.a_star_with_rl(
    start=(0, 0),
    goal=(100, 100),
    obstacles=[(50, 50)]
)

# Lider Seçimi
from lider_sec_ppo import LiderSecPPO
leader_selector = LiderSecPPO(num_rovs=6)
leader_id = leader_selector.select_leader_with_ppo(rovs_info)

# Formasyon Seçimi
from formasyon_sec_rl import FormasyonSecRL
formation_selector = FormasyonSecRL()
formation_id, formation_name, params = formation_selector.select_formation_with_hull_rl(
    rov_positions, leader_id, target_pos
)

# Model Kaydetme/Yükleme
a_star_rl.save_model('a_star_rl_model.pth')
a_star_rl.load_model('a_star_rl_model.pth')
"""

# ============================================================
# DOSYA LİSTESİ
# ============================================================

"""
RL Modelleri (6 dosya):
- a_star_rl.py                  (A* yol bulma - RL)
- formasyon_rl_enhanced.py      (Formasyon - RL)
- formasyon_sec_rl.py           (Formasyon Seçimi - RL)
- lider_sec_rl.py               (Lider Seçimi - RL)
- convex_hull_rl.py             (Convex Hull - RL)
- git_path_rl.py                (Yol Takip - RL)

PPO Modelleri (6 dosya):
- a_star_ppo.py                 (A* yol bulma - PPO)
- formasyon_sec_ppo.py          (Formasyon Seçimi - PPO)
- lider_sec_ppo.py              (Lider Seçimi - PPO)
- convex_hull_ppo.py            (Convex Hull - PPO)
- git_path_ppo.py               (Yol Takip - PPO)

Toplam: 12 yeni model dosyası
"""

print(__doc__)
