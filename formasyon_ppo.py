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


# 1. RL (PEKİŞTİRMELİ ÖĞRENME) BEYNİ - PPO ALGORİTMASI


class ActorCritic(nn.Module):
    def __init__(self, input_dim, output_dim):
        super(ActorCritic, self).__init__()
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, 64)
        self.actor = nn.Linear(64, output_dim)
        self.critic = nn.Linear(64, 1)
        self.relu = nn.ReLU()
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x):
        x = self.relu(self.fc1(x))
        x = self.relu(self.fc2(x))
        x = self.relu(self.fc3(x))
        return self.softmax(self.actor(x)), self.critic(x)

class PPOAjan:
    def __init__(self, input_dim, output_dim):
        self.policy = ActorCritic(input_dim, output_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=0.0003)
        self.eps_clip = 0.2
        self.K_epochs = 4
        self.data = [] 

    def veri_ekle(self, item):
        self.data.append(item)

    def egit(self):
        if len(self.data) < 64: return
        s_lst, a_lst, r_lst, prob_a_lst = [], [], [], []
        for transition in self.data:
            s, a, r, next_s, prob_a, done = transition
            s_lst.append(s); a_lst.append([a]); r_lst.append([r]); prob_a_lst.append([prob_a])

        s = torch.tensor(np.array(s_lst), dtype=torch.float)
        a = torch.tensor(np.array(a_lst))
        r = torch.tensor(np.array(r_lst), dtype=torch.float)
        
        for _ in range(self.K_epochs):
            probs, state_val = self.policy(s)
            advantage = r - state_val.detach()
            pi_a = probs.gather(1, a)
            ratio = torch.exp(torch.log(pi_a) - torch.tensor(prob_a_lst))
            surr1 = ratio * advantage
            surr2 = torch.clamp(ratio, 1-self.eps_clip, 1+self.eps_clip) * advantage
            loss = -torch.min(surr1, surr2) + 0.5 * nn.MSELoss()(state_val, r)
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
        self.data = []


# 2. SİMÜLASYON KURULUMU


app = Ortam()
app.sim_olustur(6, 20) # 6 ROV

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ Model yüklenemedi, AI devre dışı."); 
    beyin = None

filo = Filo()

# ROV rollerini manuel olarak ayarla
app.rovs[0].set("rol", 1) 
for i in range(1, len(app.rovs)):
    app.rovs[i].set("rol", 0)

modem = filo.otomatik_kurulum(app.rovs)
filo.manuel_kontrol_all(True) 
app.filo = filo

# --- BATARYA SİMÜLASYONU ---
# Her ROV için %100 batarya ile başlat
rov_bataryalar = [100.0] * len(app.rovs)

# --- PPO KURULUMU ---
rov_sayisi = len(app.rovs)
ppo_ajan = PPOAjan(input_dim=rov_sayisi * 4, output_dim=4) 

rl_sayac = 0
lider_hedef_z = 50.0 
lider_hedef_x = 0.0

print(f"✅ PPO Aktif Edildi. {rov_sayisi} ROV Kontrol Ediliyor. Batarya Simülasyonu Başladı.")


# 3. ANA DÖNGÜ


def update():
    global rl_sayac, lider_hedef_z, lider_hedef_x, rov_bataryalar

    try:
        dt = time.dt # Geçen süre (saniye)

        
        # A) VERİ TOPLAMA VE BATARYA HESAPLAMA
       
        anlik_veriler = [] # PPO'ya gidecek veriler
        lider_pos = app.rovs[0].position # Liderin konumu referans alınır

        for k in range(len(app.rovs)):
            # 1. HIZ AL
            hiz_vec = filo.get(k, "hiz")
            if hasattr(hiz_vec, 'length'): hiz = hiz_vec.length()
            else: hiz = np.linalg.norm(hiz_vec)

            # 2. BATARYA TÜKET (Hıza ve zamana bağlı)
            # Baz Tüketim (0.005) + Efor (Hız * 0.02)
            tuketim = (0.005 + (hiz * 0.02)) * dt * 5.0 
            rov_bataryalar[k] -= tuketim
            if rov_bataryalar[k] < 0: rov_bataryalar[k] = 0.0

            # 3. SONAR AL
            sonar = filo.get(k, "sonar")
            
            # 4. KONUM AL (Lidere göre mesafe)
            pos = app.rovs[k].position
            dist = np.linalg.norm(np.array([pos.x, pos.z]) - np.array([lider_pos.x, lider_pos.z]))

            # Verileri listeye ekle (Normalize ederek: 0.0 - 1.0 arası)
            anlik_veriler.extend([
                rov_bataryalar[k] / 100.0, # Batarya
                min(sonar, 50.0) / 50.0,   # Sonar
                hiz,                       # Hız
                dist                       # Mesafe
            ])

        # B) GAT ANALİZİ VE GÖRSELLEŞTİRME
        
        veri = app.simden_veriye()
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        
        if ai_aktif and beyin:
            try: tahminler, _, _ = beyin.analiz_et(veri)
            except: tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)

        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
        
        for i, gat_kodu in enumerate(tahminler):
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            # Etikete Batarya Yüzdesini Ekle
            app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}\nBat:%{rov_bataryalar[i]:.0f}"
        
    
        # C) PPO KARAR VE HAREKET
       
        rl_sayac += 1
        
        if app.rovs[0].position.z > lider_hedef_z - 20:
            lider_hedef_z += 40.0

        if rl_sayac % 5 == 0:
            # Hazırladığımız gerçek verileri PPO'ya veriyoruz
            state_np = np.array(anlik_veriler, dtype=np.float32)
            state_tensor = torch.from_numpy(state_np).float()
            
            # Karar Ver
            probs, _ = ppo_ajan.policy(state_tensor)
            action = Categorical(probs).sample()
            aksiyon_id = action.item()

            # Formasyon Ofsetleri
            offsets = []
            n_rov = len(app.rovs)
            if aksiyon_id == 0: # V-Şekli
                 offsets = [(0,0)] + [(pow(-1, i)*((i+1)//2)*12, -((i+1)//2)*12) for i in range(1, n_rov)]
            elif aksiyon_id == 1: # Sıra
                offsets = [(0, -i*15) for i in range(n_rov)]
            elif aksiyon_id == 2: # Kutu
                offsets = [((i%2)*15, -(i//2)*15) for i in range(n_rov)]
            elif aksiyon_id == 3: # Çember
                offsets = [(math.sin(math.radians(i*(360/n_rov)))*20, math.cos(math.radians(i*(360/n_rov)))*20) for i in range(n_rov)]

            # Hareket Uygula
            filo.git(0, lider_hedef_x, lider_hedef_z, -10) 
            
            for r_id in range(len(app.rovs)):
                if r_id != 0 and r_id < len(offsets):
                    ox, oz = offsets[r_id]
                    # Bataryası biten hareket edemez!
                    if rov_bataryalar[r_id] > 0:
                        filo.git(r_id, app.rovs[0].position.x + ox, app.rovs[0].position.z + oz, -10)

            # Ödül Hesaplama
            avg_sonar = np.mean([filo.get(k, "sonar") for k in range(len(app.rovs))])
            avg_bat = np.mean(rov_bataryalar)
            
            odul = 0
            if avg_sonar < 10.0: odul -= 10.0 # Engel Cezası
            if avg_bat < 15.0: odul -= 2.0    # Düşük Batarya Cezası
            else: odul += 1.0                 # Stabilite Ödülü
            
            # PPO Eğitimi
            prob_val = probs[aksiyon_id].item()
            ppo_ajan.veri_ekle((state_np, aksiyon_id, odul, state_np, prob_val, False))
            
            if rl_sayac % 100 == 0:
                ppo_ajan.egit()
                app.rovs[0].label.text += f"\n[EĞİTİLDİ]"

    
        # D) DETAYLI KONSOL RAPORU (Her 50 karede bir)
  
        if rl_sayac % 50 == 0:
            print(f"\n--- SİMÜLASYON DURUMU (Adım: {rl_sayac}) ---")
            print(f"{'ID':<4} | {'BATARYA':<10} | {'HIZ':<8} | {'SONAR':<8} | {'GPS (X,Z)':<15}")
            print("-" * 60)
            for k in range(len(app.rovs)):
                h = np.linalg.norm(filo.get(k, "hiz"))
                s = filo.get(k, "sonar")
                p = app.rovs[k].position
                print(f"R{k:<3} | %{rov_bataryalar[k]:>6.1f}   | {h:>6.2f}   | {s:>6.1f}   | ({p.x:.0f}, {p.z:.0f})")
            print("-" * 60)

        # GNC Sistemini Güncelle
        filo.guncelle_hepsi(tahminler)
        
    except Exception as e: 
        print(f"Update Hatası: {e}")
        pass

app.set_update_function(update)

# 4. ÇALIŞTIRMA
if __name__ == "__main__":
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        pass
    finally: 
        os.system('stty sane')
        os._exit(0)