import time
import random
import numpy as np
from .config import cfg

class AkustikModem:
    def __init__(self, rov_id, gurultu_orani=0.05, kayip_orani=0.1, gecikme=0.5):
        self.id = rov_id
        self.gurultu_orani = gurultu_orani
        self.kayip_orani = kayip_orani
        self.gecikme = gecikme
        
        self.gelen_kutusu = []
        self.rehber = {} # Diğer ROV'ların modemlerini burada tutacağız

    def rehber_guncelle(self, tum_modemler):
        """Ağdaki diğer modemleri tanımak için"""
        self.rehber = tum_modemler

    def broadcast_position(self, pozisyon_vektoru):
        """
        Lider ROV'un konumunu herkese yayması için özel fonksiyon.
        main.py ve gnc.py ile uyumluluk sağlar.
        """
        # Vec3 formatını listeye çevir (Gürültü eklemek için)
        veri = [pozisyon_vektoru.x, pozisyon_vektoru.y, pozisyon_vektoru.z]
        
        for hedef_id, hedef_modem in self.rehber.items():
            if hedef_id != self.id: # Kendine gönderme
                self.gonder(hedef_modem, veri, veri_tipi="GPS_BROADCAST")

    def gonder(self, hedef_modem, veri, veri_tipi="GENEL"):
        # 1. Paket Kaybı Simülasyonu
        if random.random() < self.kayip_orani:
            if cfg.goster_modem: 
                print(f"❌ [Modem-{self.id}] -> [Modem-{hedef_modem.id}] Paket Suda Kayboldu!")
            return False

        # 2. Gürültü Ekleme (Bozulma)
        ileti = self._gurultu_ekle(veri)

        # 3. Paketleme
        paket = {
            "kimden": self.id,
            "tip": veri_tipi,
            "veri": ileti,
            "zaman": time.time() # Gönderildiği an
        }

        # 4. Fiziksel İletim (Hedefin tampon belleğine yaz)
        hedef_modem._paket_al(paket)
        
        if cfg.goster_modem:
            print(f"📡 [Modem-{self.id}] -> [Modem-{hedef_modem.id}] Sinyal yollandı. ({veri_tipi})")
        return True

    def _paket_al(self, paket):
        # Bu fonksiyonu dışarıdan çağırmayız, 'gonder' fonksiyonu çağırır
        self.gelen_kutusu.append(paket)

    def dinle(self):
        """
        Gelen kutusunu kontrol eder. Sadece ulaşma süresi (gecikme) dolmuş paketleri verir.
        """
        if not self.gelen_kutusu: return None
        
        su_an = time.time()
        okunacaklar = []
        kalanlar = []
        
        for paket in self.gelen_kutusu:
            # Gecikme süresi doldu mu?
            if su_an - paket["zaman"] >= self.gecikme:
                okunacaklar.append(paket)
            else:
                kalanlar.append(paket)
        
        self.gelen_kutusu = kalanlar # Bekleyenleri geri koy
        return okunacaklar

    def _gurultu_ekle(self, veri):
        """Veriyi rastgele bozar"""
        if self.gurultu_orani <= 0: return veri
        
        # Eğer veri sayısal bir liste/vektör ise gürültü ekle
        if isinstance(veri, (list, np.ndarray, tuple)):
            try:
                # Veriyi numpy dizisine çevirip gürültüyle çarp
                arr = np.array(veri, dtype=float)
                noise_factor = 1.0 + np.random.uniform(-self.gurultu_orani, self.gurultu_orani, size=arr.shape)
                return list(arr * noise_factor)
            except:
                return veri # Sayısal değilse (mesajsa) dokunma
                
        return veri
