import torch
import torch.nn as nn
import torch.optim as optim
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
        self.softmax = nn.Softmax(dim=-1)
        
        # Optimizer
        self.optimizer = optim.Adam(self.parameters(), lr=learning_rate)
        
        # Hafıza (deneyimler için)
        self.hafiza = []
        
    def forward(self, x):
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
    # Eğitimi başlat
    rl_ajan = egitim_baslat(epochs=500, n_rovs=4, n_engels=10, learning_rate=0.001)
    
    # Modeli kaydet (opsiyonel)
    # torch.save(rl_ajan.state_dict(), 'formasyon_rl_model.pth')
    # print("💾 Model kaydedildi: formasyon_rl_model.pth")

