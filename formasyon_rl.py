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
        self.softmax = nn.Softmax(dim=-1)

    def forward(self, x):
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

app.set_update_function(update)

if __name__ == "__main__":
    app.run(interaktif=True)
