import torch
import torch.nn as nn
import torch.optim as optim
<<<<<<< HEAD
from torch.distributions import Categorical
import os
import math
import random

# 1. RL (PEKİŞTİRMELİ ÖĞRENME) BÖLÜMÜ (Aynı kalıyor)
class ActorCritic(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(ActorCritic, self).__init__()
        self.ortak_katman = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU()
        )
        self.actor = nn.Linear(64, output_dim)
        self.critic = nn.Linear(64, 1)
=======
import numpy as np
from FiratROVNet import senaryo


class BasitRL(nn.Module):
    """
    Basit RL Sınıfı
    - 20 giriş
    - 1 gizli katman (64 nöron)
    - 10 çıkış
    """
    def __init__(self, learning_rate=0.001):
        super(BasitRL, self).__init__()
        
        # Neural Network Katmanları
        self.giris_katman = nn.Linear(20, 64)
        self.gizli_katman = nn.ReLU()
        self.cikis_katman = nn.Linear(64, 10)
>>>>>>> bugfix/ada_engel
        self.softmax = nn.Softmax(dim=-1)
        
        # Optimizer
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)
        
        # Hafıza (deneyimler için)
        self.hafiza = []
        
    def forward(self, x):
<<<<<<< HEAD
        x = self.ortak_katman(x)
        return self.softmax(self.actor(x)), self.critic(x)

class RLAjan:
    def __init__(self, input_dim, output_dim):
        self.policy = ActorCritic(input_dim, output_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.0005)
        self.gamma = 0.99
        self.eps_clip = 0.2
        self.K_epochs = 3
        self.hafiza = []

    def hafizaya_at(self, veri):
        self.hafiza.append(veri)

    def ogren(self):
        if len(self.hafiza) < 32: return
        s_list, a_list, r_list, prob_list = [], [], [], []
        for veri in self.hafiza:
            s, a, r, next_s, prob, done = veri
            s_list.append(s); a_list.append([a]); r_list.append([r]); prob_list.append([prob])
        s = torch.tensor(np.array(s_list), dtype=torch.float)
        a = torch.tensor(np.array(a_list))
        r = torch.tensor(np.array(r_list), dtype=torch.float)
        for _ in range(self.K_epochs):
            probs, state_val = self.policy(s)
            advantage = r - state_val.detach()
            pi_a = probs.gather(1, a)
            ratio = torch.exp(torch.log(pi_a) - torch.tensor(prob_list))
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1-self.eps_clip, 1+self.eps_clip) * advantage
            loss = -torch.min(surr1, surr2) + 0.5 * nn.MSELoss()(state_val, r)
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        self.hafiza = []

# 2. SİMÜLASYON VE ORTAM KURULUMU
app = Ortam()
app.sim_olustur(6, 20) 

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    beyin = None

filo = Filo()
app.rovs[0].set("rol", 1) 
for i in range(1, len(app.rovs)): app.rovs[i].set("rol", 0)

modem = filo.otomatik_kurulum(app.rovs)
filo.manuel_kontrol_all(True) 
app.filo = filo

rov_bataryalar = [100.0] * len(app.rovs)
state_boyutu = len(app.rovs) * 4
action_boyutu = 4
ajan = RLAjan(state_boyutu, action_boyutu)

adim_sayaci = 0

# --- YENİ HEDEF MANTIĞI ---
lider_hedef_x = 0.0
lider_hedef_z = 0.0

def yeni_guvenli_hedef_sec():
    global lider_hedef_x, lider_hedef_z
    while True:
        # Havuz sınırları içinde random bir nokta (-180, 180 arası)
        deneme_x = random.uniform(-180, 180)
        deneme_z = random.uniform(-180, 180)
        
        guvenli = True
        # Ortamdaki adaları kontrol et
        if hasattr(app, 'island_positions'):
            for ada_pos in app.island_positions:
                # ada_pos -> (x, z, radius)
                d = math.sqrt((deneme_x - ada_pos[0])**2 + (deneme_z - ada_pos[1])**2)
                if d < ada_pos[2] + 25: # Ada yarıçapı + 25 birim güvenlik payı
                    guvenli = False
                    break
        
        if guvenli:
            lider_hedef_x = deneme_x
            lider_hedef_z = deneme_z
            print(f"🎯 Yeni Hedef Belirlendi: X:{lider_hedef_x:.1f}, Z:{lider_hedef_z:.1f}")
            break

# İlk hedefi belirle
yeni_guvenli_hedef_sec()

# 3. ANA DÖNGÜ
def update():
    global adim_sayaci, lider_hedef_x, lider_hedef_z

    try:
        dt = time.dt
        anlık_veriler = [] 
        lider_pos = app.rovs[0].position

        # HEDEFE ULAŞILDI MI KONTROLÜ
        mesafe_to_target = math.sqrt((lider_pos.x - lider_hedef_x)**2 + (lider_pos.z - lider_hedef_z)**2)
        if mesafe_to_target < 10.0: # 10 birim kala yeni hedefe geç
            yeni_guvenli_hedef_sec()

        for k in range(len(app.rovs)):
            hiz_vec = filo.get(k, "hiz")
            hiz = hiz_vec.length() if hasattr(hiz_vec, 'length') else np.linalg.norm(hiz_vec)
            
            tuketim = (0.005 + (hiz * 0.02)) * dt * 5.0 
            rov_bataryalar[k] -= tuketim
            if rov_bataryalar[k] < 0: rov_bataryalar[k] = 0.0
            
            sonar = filo.get(k, "sonar")
            pos = app.rovs[k].position
            dist = np.linalg.norm(np.array([pos.x, pos.z]) - np.array([lider_pos.x, lider_pos.z]))
            
            anlık_veriler.extend([
                rov_bataryalar[k] / 100.0,
                min(sonar, 50.0) / 50.0,
                hiz,
                dist
            ])

        # B) GAT (Analiz)
        veri = app.simden_veriye()
        if beyin:
            try: tahminler, _, _ = beyin.analiz_et(veri)
            except: tahminler = np.zeros(len(app.rovs), dtype=int)
        else: tahminler = np.zeros(len(app.rovs), dtype=int)

        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        for i, gat_kodu in enumerate(tahminler):
            if app.rovs[i].role == 1: app.rovs[i].color = color.red
            else: app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            app.rovs[i].label.text = f"R{i}\nBat: %{rov_bataryalar[i]:.0f}"

        # C) RL KARAR DÖNGÜSÜ
        adim_sayaci += 1
        if adim_sayaci % 5 == 0:
            state_np = np.array(anlık_veriler, dtype=np.float32)
            state_tensor = torch.from_numpy(state_np).float()

            probs, _ = ajan.policy(state_tensor)
            action = Categorical(probs).sample()
            aksiyon_id = action.item()

            # Formasyon Hesapla
            N = len(app.rovs)
            # Liderin bakış açısına (heading) göre formasyon döndürme (basitleştirilmiş)
            # Hedefe doğru olan açı
            angle_to_target = math.atan2(lider_hedef_x - lider_pos.x, lider_hedef_z - lider_pos.z)

            if aksiyon_id == 0:   # V-Şekli
                offsets = [(0,0)] + [(pow(-1, i)*((i+1)//2)*12, -((i+1)//2)*12) for i in range(1, N)]
            elif aksiyon_id == 1: # Sıra
                offsets = [(0, -i*15) for i in range(N)]
            elif aksiyon_id == 2: # Kutu
                offsets = [((i%2)*15, -(i//2)*15) for i in range(N)]
            else:                # Çember
                offsets = [(math.sin(math.radians(i*(360/N)))*20, math.cos(math.radians(i*(360/N)))*20) for i in range(N)]

            # Lideri yeni hedefe gönder
            filo.git(0, lider_hedef_x, lider_hedef_z, -10)
            
            # Takipçileri formasyona göre gönder
            for r_id in range(1, N):
                if r_id < len(offsets):
                    ox, oz = offsets[r_id]
                    # Basit rotasyon matrisi (formasyonun hedefe bakması için)
                    rx = ox * math.cos(angle_to_target) - oz * math.sin(angle_to_target)
                    rz = ox * math.sin(angle_to_target) + oz * math.cos(angle_to_target)
                    
                    if rov_bataryalar[r_id] > 0:
                        filo.git(r_id, lider_pos.x + rx, lider_pos.z + rz, -10)

            # Ödül
            avg_sonar = np.mean([filo.get(k, "sonar") for k in range(N)])
            odul = 1.0
            if avg_sonar < 12.0: odul -= 15.0
            if rov_bataryalar[0] < 10: odul -= 20.0
            
            ajan.hafizaya_at((state_np, aksiyon_id, odul, state_np, probs[aksiyon_id].item(), False))

            if adim_sayaci % 100 == 0:
                ajan.ogren()

        filo.guncelle_hepsi(tahminler)
        
    except Exception as e: 
        pass
=======
        """
        İleri geçiş (forward pass)
        x: [batch_size, 20] giriş tensörü
        Returns: [batch_size, 10] çıkış olasılıkları
        """
        x = self.giris_katman(x)
        x = self.gizli_katman(x)
        x = self.cikis_katman(x)
        return self.softmax(x)
    
    def aksiyon_sec(self, state):
        """
        Duruma göre aksiyon seç
        state: [20] numpy array veya tensor
        Returns: seçilen aksiyon indeksi (0-9)
        """
        if isinstance(state, np.ndarray):
            state = torch.FloatTensor(state).unsqueeze(0)
        
        with torch.no_grad():
            olasiliklar = self.forward(state)
            # Olasılıklara göre aksiyon seç
            aksiyon = torch.multinomial(olasiliklar, 1).item()
        
        return aksiyon
    
    def hafizaya_ekle(self, state, action, reward, next_state=None, done=False):
        """
        Deneyimi hafızaya ekle
        """
        self.hafiza.append({
            'state': state,
            'action': action,
            'reward': reward,
            'next_state': next_state,
            'done': done
        })
    
    def ogren(self, gamma=0.99):
        """
        REINFORCE algoritması ile öğrenme
        gamma: indirim faktörü (discount factor)
        """
        if len(self.hafiza) == 0:
            return
        
        # Hafızadaki tüm deneyimleri işle
        states = []
        actions = []
        rewards = []
        
        for deneyim in self.hafiza:
            states.append(deneyim['state'])
            actions.append(deneyim['action'])
            rewards.append(deneyim['reward'])
        
        # Tensor'lara dönüştür
        states = torch.FloatTensor(np.array(states))
        actions = torch.LongTensor(actions)
        rewards = np.array(rewards)
        
        # Gecikmiş ödülleri hesapla (discounted rewards)
        gecikmis_oduller = []
        G = 0
        for r in reversed(rewards):
            G = r + gamma * G
            gecikmis_oduller.insert(0, G)
        
        gecikmis_oduller = torch.FloatTensor(gecikmis_oduller)
        # Normalize et (opsiyonel, daha stabil eğitim için)
        gecikmis_oduller = (gecikmis_oduller - gecikmis_oduller.mean()) / (gecikmis_oduller.std() + 1e-8)
        
        # Policy gradient hesapla
        olasiliklar = self.forward(states)
        secilen_olasiliklar = olasiliklar.gather(1, actions.unsqueeze(1)).squeeze(1)
        
        # Loss: -log(pi(a|s)) * G (gradient ascent için negatif)
        loss = -(torch.log(secilen_olasiliklar + 1e-8) * gecikmis_oduller).mean()
        
        # Optimize et
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Hafızayı temizle
        self.hafiza = []
        
        return loss.item()
    
    def hafizayi_temizle(self):
        """Hafızayı temizle"""
        self.hafiza = []


# Ödül fonksiyonu (basit örnek - ihtiyacınıza göre özelleştirebilirsiniz)
def odul_hesapla(batarya_degerleri, sonar_degerleri, gps_koordinatlari):
    """
    Basit ödül fonksiyonu
    - Batarya yüksekse ödül artar
    - Sonar mesafesi yüksekse (engel yoksa) ödül artar
    - ROV'lar birbirine çok yakınsa ceza
    """
    odul = 0.0
    
    # Batarya ödülü (ortalama batarya seviyesi)
    ortalama_batarya = np.mean(batarya_degerleri)
    odul += ortalama_batarya * 0.3
    
    # Sonar ödülü (engel mesafesi yüksekse iyi)
    # Sonar -1 ise engel yok demektir, bu durumda maksimum ödül
    sonar_odul = 0.0
    for sonar in sonar_degerleri:
        if sonar == -1:
            sonar_odul += 1.0  # Engel yok, maksimum ödül
        else:
            # Sonar değerini normalize et (0-1 arası, yüksek değer = iyi)
            # Örnek: 50 birim mesafe varsa, normalize edilmiş değer ~0.5 olabilir
            normalized_sonar = min(sonar / 50.0, 1.0)  # 50 birim = maksimum
            sonar_odul += normalized_sonar
    odul += (sonar_odul / len(sonar_degerleri)) * 0.3
    
    # ROV'lar arası mesafe kontrolü (çok yakınsa ceza)
    if len(gps_koordinatlari) >= 2:
        min_mesafe = float('inf')
        for i in range(len(gps_koordinatlari)):
            for j in range(i+1, len(gps_koordinatlari)):
                pos1 = gps_koordinatlari[i]
                pos2 = gps_koordinatlari[j]
                mesafe = np.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)
                min_mesafe = min(min_mesafe, mesafe)
        
        # Minimum mesafe 10 birimden azsa ceza
        if min_mesafe < 10:
            odul -= 0.2
    
    return odul


def veri_topla():
    """
    4 ROV'dan veri toplar ve 20 boyutlu state vektörü oluşturur.
    
    Returns:
        state: [20] numpy array
            [batarya_0, gps_x_0, gps_y_0, gps_z_0, sonar_0,
             batarya_1, gps_x_1, gps_y_1, gps_z_1, sonar_1,
             batarya_2, gps_x_2, gps_y_2, gps_z_2, sonar_2,
             batarya_3, gps_x_3, gps_y_3, gps_z_3, sonar_3]
    """
    state = []
    batarya_degerleri = []
    sonar_degerleri = []
    gps_koordinatlari = []
    
    for rov_id in range(4):
        # Batarya (0-1 arası)
        batarya = senaryo.get(rov_id, "batarya")
        if batarya is None:
            batarya = 0.5  # Varsayılan değer
        batarya_degerleri.append(batarya)
        state.append(batarya)
        
        # GPS (x, y, z)
        gps = senaryo.get(rov_id, "gps")
        if gps is None:
            gps = np.array([0.0, 0.0, 0.0])
        else:
            gps = np.array(gps)
        gps_koordinatlari.append(gps)
        
        # GPS koordinatlarını normalize et (örnek: -200 ile 200 arası -> 0-1 arası)
        # Havuz genişliği 200 olduğu için koordinatlar -200 ile 200 arasında olabilir
        normalized_x = (gps[0] + 200) / 400.0  # -200 -> 0, 200 -> 1
        normalized_y = (gps[1] + 200) / 400.0
        normalized_z = (gps[2] + 200) / 400.0
        
        state.extend([normalized_x, normalized_y, normalized_z])
        
        # Sonar (0-1 arası normalize edilmiş değer)
        sonar = senaryo.get(rov_id, "sonar")
        if sonar is None or sonar == -1:
            # Engel yok, normalize edilmiş değer = 1.0 (maksimum güvenlik)
            normalized_sonar = 1.0
            sonar_degerleri.append(-1)  # Ham değer için -1 sakla
        else:
            # Sonar değerini normalize et (0-50 birim arası -> 0-1 arası)
            # Mesafe ne kadar büyükse o kadar iyi (1.0'a yakın)
            normalized_sonar = min(sonar / 50.0, 1.0)
            sonar_degerleri.append(sonar)  # Ham değeri sakla
        
        state.append(normalized_sonar)
    
    state = np.array(state, dtype=np.float32)
    
    return state, batarya_degerleri, sonar_degerleri, gps_koordinatlari
>>>>>>> bugfix/ada_engel


# Eğitim fonksiyonu
def egitim_baslat(epochs=500, n_rovs=4, n_engels=10, learning_rate=0.001):
    """
    500 epoch eğitim başlatır.
    
    Args:
        epochs: Eğitim epoch sayısı (varsayılan: 500)
        n_rovs: ROV sayısı (varsayılan: 4)
        n_engels: Engel sayısı (varsayılan: 10)
        learning_rate: Öğrenme oranı (varsayılan: 0.001)
    """
    # RL Ajanını oluştur
    rl_ajan = BasitRL(learning_rate=learning_rate)
    
    # İlk kurulum (yavaş - ortam oluşturulur)
    print("🚀 Senaryo oluşturuluyor...")
    senaryo.uret(n_rovs=n_rovs, n_engels=n_engels)
    print("✅ Senaryo hazır!")
    
    # 500 Epoch Eğitim
    print(f"\n🎯 Eğitim Başlıyor ({epochs} Epoch)...")
    print("=" * 60)
    
    for epoch in range(epochs):
        # Senaryo pozisyonlarını güncelle (hızlı - sadece pozisyonlar değişir)
        senaryo.uret()  # Aynı sayılar, farklı koordinatlar
        
        # Veri topla
        state, batarya_degerleri, sonar_degerleri, gps_koordinatlari = veri_topla()
        
        # Aksiyon seç (formasyon seçimi: 0-9 arası)
        aksiyon = rl_ajan.aksiyon_sec(state)
        
        # Ödül hesapla
        odul = odul_hesapla(batarya_degerleri, sonar_degerleri, gps_koordinatlari)
        
        # Hafızaya ekle
        rl_ajan.hafizaya_ekle(state, aksiyon, odul)
        
        # Her 32 deneyimde bir öğren (veya epoch sonunda)
        if len(rl_ajan.hafiza) >= 32:
            loss = rl_ajan.ogren()
            if epoch % 50 == 0:  # Her 50 epoch'ta bir loss yazdır
                print(f"   🔹 Epoch {epoch}/{epochs} | Loss: {loss:.4f} | Ödül: {odul:.4f} | Aksiyon: {aksiyon}")
        
        # Epoch sonunda kalan deneyimleri de öğren
        if epoch == epochs - 1 and len(rl_ajan.hafiza) > 0:
            loss = rl_ajan.ogren()
            print(f"   🔹 Epoch {epoch+1}/{epochs} | Loss: {loss:.4f} | Ödül: {odul:.4f} | Aksiyon: {aksiyon}")
    
    print("\n✅ Eğitim tamamlandı!")
    print("=" * 60)
    
    return rl_ajan


# Kullanım örneği
if __name__ == "__main__":
<<<<<<< HEAD
    app.run(interaktif=True)
=======
    # Eğitimi başlat
    rl_ajan = egitim_baslat(epochs=500, n_rovs=4, n_engels=10, learning_rate=0.001)
    
    # Modeli kaydet (opsiyonel)
    # torch.save(rl_ajan.state_dict(), 'formasyon_rl_model.pth')
    # print("💾 Model kaydedildi: formasyon_rl_model.pth")

>>>>>>> bugfix/ada_engel
