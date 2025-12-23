from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Categorical
import os
import math


# 1. RL (PEKİŞTİRMELİ ÖĞRENME) 


class ActorCritic(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(ActorCritic, self).__init__()
        # Aktör (Karar Verici) ve Kritik (Durum Değerlendirici) Ağları
        self.ortak_katman = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU()
        )
        self.actor = nn.Linear(64, output_dim) # Hangi formasyonu seçecek? (Olasılıklar)
        self.critic = nn.Linear(64, 1)         # Bu durum ne kadar iyi? (Puan)
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x):
        x = self.ortak_katman(x)
        return self.softmax(self.actor(x)), self.critic(x)

class RLAjan:
    def __init__(self, input_dim, output_dim):
        self.policy = ActorCritic(input_dim, output_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.0005)
        self.gamma = 0.99      # Gelecek ödüllerin önemi
        self.eps_clip = 0.2    # Modelin çok ani değişmesini engelleme
        self.K_epochs = 3      # Her veriyle kaç kere eğitileceği
        self.hafiza = []       # Deneyim havuzu

    def hafizaya_at(self, veri):
        self.hafiza.append(veri)

    def ogren(self):
        if len(self.hafiza) < 32: return # Yeterince veri yoksa öğrenme
        
        s_list, a_list, r_list, prob_list = [], [], [], []
        for veri in self.hafiza:
            s, a, r, next_s, prob, done = veri
            s_list.append(s); a_list.append([a]); r_list.append([r]); prob_list.append([prob])

        s = torch.tensor(np.array(s_list), dtype=torch.float)
        a = torch.tensor(np.array(a_list))
        r = torch.tensor(np.array(r_list), dtype=torch.float)

        # PPO Güncelleme Döngüsü
        for _ in range(self.K_epochs):
            probs, state_val = self.policy(s)
            advantage = r - state_val.detach() # Beklenenden ne kadar iyi ödül aldık?
            
            pi_a = probs.gather(1, a)
            ratio = torch.exp(torch.log(pi_a) - torch.tensor(prob_list))
            
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1-self.eps_clip, 1+self.eps_clip) * advantage
            loss = -torch.min(surr1, surr2) + 0.5 * nn.MSELoss()(state_val, r)
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        
        self.hafiza = [] # Hafızayı temizle


# 2. SİMÜLASYON VE ORTAM KURULUMU


app = Ortam()
app.sim_olustur(6, 20) # 6 ROV ile ortamı kur

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ GAT Modeli yüklenemedi, AI devre dışı."); beyin = None

filo = Filo()
# ROV Rollerini Ayarla: 0 numara Lider, diğerleri Takipçi
app.rovs[0].set("rol", 1) 
for i in range(1, len(app.rovs)): app.rovs[i].set("rol", 0)

modem = filo.otomatik_kurulum(app.rovs)
filo.manuel_kontrol_all(True) 
app.filo = filo

# --- BATARYA SİMÜLASYONU DEĞİŞKENLERİ ---
# Her ROV için %100 batarya ile başlatıyoruz
rov_bataryalar = [100.0] * len(app.rovs)

# --- RL KURULUMU ---
state_boyutu = len(app.rovs) * 4  # Her ROV için: Batarya, Sonar, Hız, Mesafe
action_boyutu = 4                 # Formasyonlar: V, Sıra, Kutu, Çember
ajan = RLAjan(state_boyutu, action_boyutu)

adim_sayaci = 0
lider_hedef_z = 50.0 
lider_hedef_x = 0.0

print(f"✅ RL Sistemi Hazır. Batarya Simülasyonu Aktif.")


# 3. ANA DÖNGÜ (RL MANTIĞI VE VERİ ENTEGRASYONU)


def update():
    global adim_sayaci, lider_hedef_z

    try:
        dt = time.dt # Geçen süre (saniye)

      
        # A) VERİ TOPLAMA VE BATARYA SİMÜLASYONU
       
        
        # RL Modelini besleyecek olan anlık veriler listesi
        anlık_veriler = [] 
        
        lider_pos = app.rovs[0].position

        for k in range(len(app.rovs)):
            # 1. HIZ HESAPLA (Vektör büyüklüğü)
            hiz_vec = filo.get(k, "hiz")
            if hasattr(hiz_vec, 'length'): hiz = hiz_vec.length()
            else: hiz = np.linalg.norm(hiz_vec)
            
            # 2. BATARYA TÜKETİMİ
            # Formül: Baz Tüketim + (Hız * Efor) * Zaman
            # Hızlı gidenin şarjı daha çabuk biter.
            tuketim = (0.005 + (hiz * 0.02)) * dt * 5.0 # 5.0 katsayısı testi hızlandırmak için
            rov_bataryalar[k] -= tuketim
            if rov_bataryalar[k] < 0: rov_bataryalar[k] = 0.0
            
            # 3. SONAR VE KONUM
            sonar = filo.get(k, "sonar")
            pos = app.rovs[k].position
            
            # 4. LİDERE UZAKLIK (GPS Mantığı)
            dist = np.linalg.norm(np.array([pos.x, pos.z]) - np.array([lider_pos.x, lider_pos.z]))
            
            # Verileri RL için kaydet (Normalize ederek)
            anlık_veriler.extend([
                rov_bataryalar[k] / 100.0, # Batarya (0-1 arası)
                min(sonar, 50.0) / 50.0,   # Sonar (0-1 arası)
                hiz,                       # Hız
                dist                       # Mesafe
            ])

       
        # B) GAT VE GÖRSELLEŞTİRME
     
        veri = app.simden_veriye()
        
        if getattr(cfg, 'ai_aktif', True) and beyin:
            try: tahminler, _, _ = beyin.analiz_et(veri)
            except: tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)

        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        for i, gat_kodu in enumerate(tahminler):
            if app.rovs[i].role == 1: app.rovs[i].color = color.red
            else: app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            # Ekranda Batarya Durumunu Göster
            app.rovs[i].label.text = f"R{i}\nBat: %{rov_bataryalar[i]:.0f}"

       
        # C) RL KARAR DÖNGÜSÜ
        
        adim_sayaci += 1
        
        # Lider Hedef İlerlemesi
        if lider_pos.z > lider_hedef_z - 20: lider_hedef_z += 40.0

        if adim_sayaci % 5 == 0:
            
            # 1. GÖZLEM: Topladığımız 'anlık_veriler' listesini kullanıyoruz
            state_np = np.array(anlık_veriler, dtype=np.float32)
            state_tensor = torch.from_numpy(state_np).float()

            # 2. KARAR
            probs, _ = ajan.policy(state_tensor)
            m = Categorical(probs)
            action = m.sample()
            aksiyon_id = action.item()

            # Formasyon Hesapla
            offsets = []
            N = len(app.rovs)
            if aksiyon_id == 0:   # V-Şekli
                offsets = [(0,0)] + [(pow(-1, i)*((i+1)//2)*12, -((i+1)//2)*12) for i in range(1, N)]
            elif aksiyon_id == 1: # Sıra
                offsets = [(0, -i*15) for i in range(N)]
            elif aksiyon_id == 2: # Kutu
                offsets = [((i%2)*15, -(i//2)*15) for i in range(N)]
            elif aksiyon_id == 3: # Çember
                offsets = [(math.sin(math.radians(i*(360/N)))*20, math.cos(math.radians(i*(360/N)))*20) for i in range(N)]

            # Uygula
            filo.git(0, lider_hedef_x, lider_hedef_z, -10)
            for r_id in range(1, N):
                if r_id < len(offsets):
                    ox, oz = offsets[r_id]
                    # Batarya varsa hareket etsin
                    if rov_bataryalar[r_id] > 0:
                        filo.git(r_id, lider_pos.x + ox, lider_pos.z + oz, -10)

            # 3. ÖDÜL MEKANİZMASI
            avg_sonar = np.mean([filo.get(k, "sonar") for k in range(N)])
            avg_bat = np.mean(rov_bataryalar)
            
            odul = 0
            if avg_sonar < 10.0: odul -= 10.0 # Çarpışma riski
            if avg_bat < 15.0: odul -= 2.0    # Kritik batarya cezası
            else: odul += 1.0                 # Stabil durum ödülü
            
            # Hafızaya At
            prob_val = probs[aksiyon_id].item()
            ajan.hafizaya_at((state_np, aksiyon_id, odul, state_np, prob_val, False))

            # 4. EĞİTİM
            if adim_sayaci % 100 == 0:
                ajan.ogren()

  
        # D) DETAYLI KONSOL RAPORU (50 adımda bir)
      
        if adim_sayaci % 50 == 0:
            print(f"\n--- DURUM RAPORU (Adım: {adim_sayaci}) ---")
            print(f"{'ID':<4} | {'BATARYA':<10} | {'HIZ':<8} | {'SONAR':<8} | {'GPS (X,Z)':<15}")
            print("-" * 60)
            for k in range(len(app.rovs)):
                h = np.linalg.norm(filo.get(k, "hiz"))
                s = filo.get(k, "sonar")
                p = app.rovs[k].position
                print(f"R{k:<3} | %{rov_bataryalar[k]:>6.2f}   | {h:>6.2f}   | {s:>6.1f}   | ({p.x:.0f}, {p.z:.0f})")
            print("-" * 60)

        filo.guncelle_hepsi(tahminler)
        
    except Exception as e: 
        print(f"Hata: {e}")
        pass

app.set_update_function(update)

if __name__ == "__main__":
    try: app.run(interaktif=True)
    except KeyboardInterrupt: pass
    finally: os.system('stty sane'); os._exit(0)