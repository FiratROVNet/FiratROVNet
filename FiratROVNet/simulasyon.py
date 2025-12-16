from ursina import *
import numpy as np
import random
import threading
import code
import sys
import torch
    
from .config import cfg # <-- BU SATIRI EKLE

  

# --- FİZİK SABİTLERİ ---
SURTUNME_KATSAYISI = 0.95
HIZLANMA_CARPANI = 30  # Artırıldı: 0.5 -> 5.0 (daha hızlı hareket için)
KALDIRMA_KUVVETI = 2.0
BATARYA_SOMURME_KATSAYISI = 0.001  # Batarya sömürme katsayısı (gerçekçi değer: maksimum güçte ~66 saniye dayanır)



class ROV(Entity):
    def __init__(self, rov_id, **kwargs):
        super().__init__()
        self.model = 'cube'
        self.color = color.orange # Turuncu her zaman görünür
        self.scale = (1.5, 0.8, 2.5)
        self.collider = 'box'
        self.unlit = True 
        
        if 'position' in kwargs: self.position = kwargs['position']
        else: self.position = (0, -5, 0)

        self.label = Text(text=f"ROV-{rov_id}", parent=self, y=1.5, scale=5, billboard=True, color=color.white)
        
        self.id = rov_id
        self.velocity = Vec3(0, 0, 0)
        self.battery = 1.0  # Batarya 0-1 arası (1.0 = %100 dolu)
        self.role = 0
        self.calistirilan_guc = 0.0  # ROV'un çalıştırdığı güç (0.0-1.0 arası) 
        
        self.sensor_config = {
            "engel_mesafesi": 20.0,
            "iletisim_menzili": 35.0,
            "min_pil_uyarisi": 10.0,
            "kacinma_mesafesi": 8.0  # Kaçınma mesafesi (ROV'lar ve engeller için)
        }
        self.environment_ref = None
        
        # Manuel hareket kontrolü (sürekli hareket için)
        self.manuel_hareket = {
            'yon': None,  # 'ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur'
            'guc': 0.0    # 0.0 - 1.0 arası güç
        }
        
        # Engel tespit bilgisi (kesikli çizgi için)
        self.tespit_edilen_engel = None  # En yakın engel referansı
        self.engel_mesafesi = 999.0  # En yakın engel mesafesi
        self.engel_cizgi = None  # Kesikli çizgi entity'si
        
        # Sonar iletişim bilgisi (ROV'lar arası kesikli çizgi için)
        self.iletisim_rovlari = {}  # {rov_id: {'mesafe': float, 'cizgi': Entity, 'yuzey_iletisimi': bool}}
        
        # İletişim durumu (liderle iletişim var mı?)
        self.lider_ile_iletisim = False  # Liderle iletişim durumu
        self.yuzeyde = False  # Yüzeyde mi? (y >= 0) 

    def update(self):
        # Manuel hareket kontrolü (sürekli hareket için)
        if self.manuel_hareket['yon'] is not None and self.manuel_hareket['guc'] > 0:
            if self.manuel_hareket['yon'] == 'dur':
                self.velocity *= 0.8  # Yavaşça dur (momentum korunumu)
                if self.velocity.length() < 0.1:
                    self.velocity = Vec3(0, 0, 0)
                    self.manuel_hareket['yon'] = None
                    self.manuel_hareket['guc'] = 0.0
            else:
                # Sürekli hareket: move metodunu çağır
                yon = self.manuel_hareket['yon']
                guc = self.manuel_hareket['guc']
                self.move(yon, guc)
        
        # Engel tespiti (her zaman çalışır, manuel kontrol olsun olmasın)
        if self.environment_ref:
            self._engel_tespiti()
        
        # Sonar iletişim tespiti (ROV'lar arası kesikli çizgi)
        if self.environment_ref:
            self._sonar_iletisim()
        
        # Yüzey durumu güncelle
        self.yuzeyde = self.y >= 0
        
        # Liderle iletişim kontrolü (takipçi ROV'lar için)
        if self.role == 0 and self.environment_ref:  # Takipçi ise
            self._lider_iletisim_kontrolu()
        
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= SURTUNME_KATSAYISI
        
        # Simülasyon sınır kontrolü (ROV'ların dışarı çıkmasını önle)
        if self.environment_ref:
            havuz_genisligi = getattr(self.environment_ref, 'havuz_genisligi', 200)
            havuz_yari_genislik = havuz_genisligi / 2
            
            # X ve Z sınırları
            if abs(self.x) > havuz_yari_genislik:
                self.x = np.sign(self.x) * havuz_yari_genislik
                self.velocity.x = 0  # Sınırda durdur
            
            if abs(self.z) > havuz_yari_genislik:
                self.z = np.sign(self.z) * havuz_yari_genislik
                self.velocity.z = 0  # Sınırda durdur
        
        if self.role == 1: # Lider
            if self.y < 0:
                self.velocity.y += KALDIRMA_KUVVETI * time.dt
                if self.y > -0.5: self.velocity.y *= 0.5
            if self.y < -2: self.y = -2
            if self.y > 0.5: 
                self.y = 0.5
                self.velocity.y = 0
        else: # Takipçi
            if self.y > 0: 
                self.y = 0
                self.velocity.y = 0
            if self.y < -100: 
                self.y = -100
                self.velocity.y = 0

        if self.velocity.length() > 0.01: 
            self.battery -= BATARYA_SOMURME_KATSAYISI * time.dt
        
        # Yakınlaşma önleme (10 metre mesafede uzaklaşma)
        if self.environment_ref:
            self._yaklasma_onleme()
        
        # Çarpışma kontrolü
        if self.environment_ref:
            self._carpisma_kontrolu()

    def move(self, komut, guc=1.0):
        # Batarya bitmişse hareket ettirme
        if self.battery <= 0:
            return
        thrust = guc * HIZLANMA_CARPANI * time.dt

        if komut == "ileri":  self.velocity.z += thrust
        elif komut == "geri": self.velocity.z -= thrust
        elif komut == "sag":  self.velocity.x += thrust
        elif komut == "sol":  self.velocity.x -= thrust
        elif komut == "cik":  self.velocity.y += thrust 
        elif komut == "bat":  
            if self.role == 1: pass
            else: self.velocity.y -= thrust 
        elif komut == "dur":
            self.velocity = Vec3(0,0,0)

    def set(self, ayar_adi, deger):
        if ayar_adi == "rol":
            self.role = int(deger)
            if self.role == 1:
                self.color = color.red
                self.label.text = f"LIDER-{self.id}"
                print(f"✅ ROV-{self.id} artık LİDER.")
            else:
                self.color = color.orange
                self.label.text = f"ROV-{self.id}"
                print(f"✅ ROV-{self.id} artık TAKİPÇİ.")
        elif ayar_adi in self.sensor_config: 
            self.sensor_config[ayar_adi] = deger

    def get(self, veri_tipi):
        if veri_tipi == "gps": 
            return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": 
            return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": 
            return self.battery
        elif veri_tipi == "rol": 
            return self.role
        elif veri_tipi == "renk": 
            return self.color
        elif veri_tipi == "sensör" or veri_tipi == "sensor":
            return self.sensor_config.copy()
        elif veri_tipi == "engel_mesafesi": 
            return self.sensor_config.get("engel_mesafesi")
        elif veri_tipi == "iletisim_menzili": 
            return self.sensor_config.get("iletisim_menzili")
        elif veri_tipi == "min_pil_uyarisi": 
            return self.sensor_config.get("min_pil_uyarisi")
        elif veri_tipi == "kacinma_mesafesi":
            return self.sensor_config.get("kacinma_mesafesi")
        elif veri_tipi == "sonar":
            min_dist = 999.0
            if self.environment_ref:
                for engel in self.environment_ref.engeller:
                    avg_scale = (engel.scale_x + engel.scale_z) / 2
                    d = distance(self, engel) - (avg_scale / 2)
                    if d < min_dist: min_dist = d
            menzil = self.sensor_config["engel_mesafesi"]
            return min_dist if min_dist < menzil else -1
        return None
    
    def _engel_tespiti(self):
        """
        Engelleri tespit eder ve kesikli çizgi çizer.
        Manuel kontrol olsun olmasın her zaman çalışır.
        """
        if not self.environment_ref:
            return
        
        min_mesafe = 999.0
        en_yakin_engel = None
        
        # Tüm engelleri kontrol et
        for engel in self.environment_ref.engeller:
            mesafe = distance(self.position, engel.position)
            # Engel boyutunu dikkate al
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            gercek_mesafe = mesafe - engel_yari_cap
            
            if gercek_mesafe < min_mesafe:
                min_mesafe = gercek_mesafe
                en_yakin_engel = engel
        
        # Sensör menzili kontrolü
        engel_mesafesi_limit = self.sensor_config.get("engel_mesafesi", 20.0)
        
        # Eğer engel tespit edildiyse
        if en_yakin_engel and min_mesafe < engel_mesafesi_limit:
            self.tespit_edilen_engel = en_yakin_engel
            self.engel_mesafesi = min_mesafe
            
            # Kesikli çizgi çiz (veya güncelle)
            self._kesikli_cizgi_ciz(en_yakin_engel, min_mesafe)
        else:
            # Engel tespit edilmediyse çizgiyi kaldır
            self.tespit_edilen_engel = None
            self.engel_mesafesi = 999.0
            if self.engel_cizgi:
                destroy(self.engel_cizgi)
                self.engel_cizgi = None
    
    def _kesikli_cizgi_ciz(self, engel, mesafe):
        """
        ROV'dan engele doğru kesikli çizgi çizer.
        """
        # Eski çizgiyi kaldır
        if self.engel_cizgi:
            if hasattr(self.engel_cizgi, 'children'):
                for child in self.engel_cizgi.children:
                    destroy(child)
            destroy(self.engel_cizgi)
        
        # Çizgi rengi: mesafeye göre (yakın = kırmızı, uzak = sarı)
        if mesafe < 5.0:
            cizgi_rengi = color.red
        elif mesafe < 10.0:
            cizgi_rengi = color.orange
        else:
            cizgi_rengi = color.yellow
        
        # Kesikli çizgi için noktalar oluştur
        baslangic = self.position
        bitis = engel.position
        yon = (bitis - baslangic)
        if yon.length() == 0:
            return
        yon = yon.normalized()
        toplam_mesafe = distance(baslangic, bitis)
        
        # Kesikli çizgi parçaları (her 2 birimde bir parça)
        parca_uzunlugu = 2.0
        bosluk_uzunlugu = 1.0
        
        # Ana çizgi entity'si (parçaları tutmak için)
        self.engel_cizgi = Entity()
        
        # Çizgi parçalarını oluştur
        mevcut_pozisyon = 0.0
        
        while mevcut_pozisyon < toplam_mesafe:
            # Parça başlangıcı
            parca_baslangic = baslangic + yon * mevcut_pozisyon
            
            # Parça bitişi
            parca_bitis_uzunlugu = min(parca_uzunlugu, toplam_mesafe - mevcut_pozisyon)
            if parca_bitis_uzunlugu <= 0:
                break
            
            parca_bitis = parca_baslangic + yon * parca_bitis_uzunlugu
            
            # Parça entity'si oluştur (basit küp)
            parca = Entity(
                model='cube',
                position=(parca_baslangic + parca_bitis) / 2,
                scale=(0.15, 0.15, parca_bitis_uzunlugu),
                color=cizgi_rengi,
                parent=self.engel_cizgi,
                unlit=True
            )
            
            # Yönlendirme (basit yöntem)
            parca.look_at(parca_bitis, up=Vec3(0, 1, 0))
            
            # Sonraki parça için pozisyon güncelle
            mevcut_pozisyon += parca_uzunlugu + bosluk_uzunlugu
    
    def _sonar_iletisim(self):
        """
        Yakın ROV'ları tespit eder ve aralarında kesikli çizgi çizer (sonar iletişimi).
        Manuel kontrol olsun olmasın her zaman çalışır.
        
        YENİ: Yüzey iletişimi desteği - yüzeydeki ROV'lar arası iletişim sınırsızdır.
        """
        if not self.environment_ref:
            return
        
        # İletişim menzili (su altı için)
        iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
        
        # Yüzey kontrolü (y >= 0 ise yüzeyde sayılır)
        self_yuzeyde = self.y >= 0
        
        # Mevcut iletişimdeki ROV'ları kontrol et
        aktif_iletisim_rovlari = {}
        
        # Tüm ROV'ları kontrol et (sadece kendinden büyük ID'li ROV'lara çizgi çiz, çift çizgiyi önlemek için)
        for diger_rov in self.environment_ref.rovs:
            if diger_rov.id == self.id:
                continue
            
            # Sadece kendinden büyük ID'li ROV'lara çizgi çiz (her çift için tek çizgi)
            if diger_rov.id <= self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            diger_rov_yuzeyde = diger_rov.y >= 0
            
            # YÜZEY İLETİŞİMİ: Her iki ROV da yüzeydeyse iletişim sınırsız
            if self_yuzeyde and diger_rov_yuzeyde:
                # Yüzeydeki ROV'lar arası iletişim sınırsız (radyo dalgaları)
                aktif_iletisim_rovlari[diger_rov.id] = {
                    'rov': diger_rov,
                    'mesafe': mesafe,
                    'yuzey_iletisimi': True  # Yüzey iletişimi işareti
                }
            # SU ALTI İLETİŞİMİ: Normal menzil kontrolü
            elif mesafe < iletisim_menzili:
                aktif_iletisim_rovlari[diger_rov.id] = {
                    'rov': diger_rov,
                    'mesafe': mesafe,
                    'yuzey_iletisimi': False
                }
        
        # Eski iletişim çizgilerini temizle (artık iletişimde olmayanlar)
        silinecek_rovlar = []
        for rov_id, iletisim_bilgisi in self.iletisim_rovlari.items():
            if rov_id not in aktif_iletisim_rovlari:
                # İletişim koptu, çizgiyi kaldır
                if iletisim_bilgisi.get('cizgi'):
                    destroy(iletisim_bilgisi['cizgi'])
                silinecek_rovlar.append(rov_id)
        
        for rov_id in silinecek_rovlar:
            del self.iletisim_rovlari[rov_id]
        
        # Yeni iletişim çizgileri çiz veya güncelle
        for rov_id, iletisim_bilgisi in aktif_iletisim_rovlari.items():
            diger_rov = iletisim_bilgisi['rov']
            mesafe = iletisim_bilgisi['mesafe']
            yuzey_iletisimi = iletisim_bilgisi.get('yuzey_iletisimi', False)
            
            # Eğer zaten iletişim varsa güncelle, yoksa yeni çiz
            if rov_id in self.iletisim_rovlari:
                # Mevcut çizgiyi güncelle
                if self.iletisim_rovlari[rov_id].get('cizgi'):
                    destroy(self.iletisim_rovlari[rov_id]['cizgi'])
            
            # Yeni çizgi çiz (yüzey iletişimi için özel stil)
            cizgi = self._rov_arasi_cizgi_ciz(diger_rov, mesafe, yuzey_iletisimi=yuzey_iletisimi)
            
            # İletişim bilgisini güncelle
            self.iletisim_rovlari[rov_id] = {
                'rov': diger_rov,
                'mesafe': mesafe,
                'cizgi': cizgi,
                'yuzey_iletisimi': yuzey_iletisimi
            }
    
    def _rov_arasi_cizgi_ciz(self, diger_rov, mesafe, yuzey_iletisimi=False):
        """
        İki ROV arasında kesikli çizgi çizer (sonar iletişimi veya yüzey iletişimi).
        
        Args:
            diger_rov: İletişim kurulan diğer ROV
            mesafe: İki ROV arasındaki mesafe
            yuzey_iletisimi: True ise yüzey iletişimi (radyo dalgaları), False ise su altı (sonar)
        
        Returns:
            Entity: Çizgi entity'si
        """
        # YÜZEY İLETİŞİMİ: Yeşil renk (radyo dalgaları)
        if yuzey_iletisimi:
            cizgi_rengi = color.green
        else:
            # SU ALTI İLETİŞİMİ: Mesafeye göre renk (yakın = mavi, uzak = cyan)
            iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
            mesafe_orani = mesafe / iletisim_menzili
            
            if mesafe_orani < 0.3:  # Çok yakın
                cizgi_rengi = color.blue
            elif mesafe_orani < 0.6:  # Orta mesafe
                cizgi_rengi = color.cyan
            else:  # Uzak ama hala menzil içinde
                cizgi_rengi = color.rgb(100, 200, 255)  # Açık mavi
        
        # Kesikli çizgi için noktalar oluştur
        baslangic = self.position
        bitis = diger_rov.position
        yon = (bitis - baslangic)
        if yon.length() == 0:
            return None
        yon = yon.normalized()
        toplam_mesafe = distance(baslangic, bitis)
        
        # Kesikli çizgi parçaları (her 1.5 birimde bir parça, daha ince)
        parca_uzunlugu = 1.5
        bosluk_uzunlugu = 0.8
        
        # Ana çizgi entity'si (parçaları tutmak için)
        cizgi_entity = Entity()
        
        # Çizgi parçalarını oluştur
        mevcut_pozisyon = 0.0
        
        while mevcut_pozisyon < toplam_mesafe:
            # Parça başlangıcı
            parca_baslangic = baslangic + yon * mevcut_pozisyon
            
            # Parça bitişi
            parca_bitis_uzunlugu = min(parca_uzunlugu, toplam_mesafe - mevcut_pozisyon)
            if parca_bitis_uzunlugu <= 0:
                break
            
            parca_bitis = parca_baslangic + yon * parca_bitis_uzunlugu
            
            # Parça entity'si oluştur (daha ince, iletişim çizgisi için)
            parca = Entity(
                model='cube',
                position=(parca_baslangic + parca_bitis) / 2,
                scale=(0.1, 0.1, parca_bitis_uzunlugu),
                color=cizgi_rengi,
                parent=cizgi_entity,
                unlit=True
            )
            
            # Yönlendirme
            parca.look_at(parca_bitis, up=Vec3(0, 1, 0))
            
            # Sonraki parça için pozisyon güncelle
            mevcut_pozisyon += parca_uzunlugu + bosluk_uzunlugu
        
        return cizgi_entity
    
    def _lider_iletisim_kontrolu(self):
        """
        Takipçi ROV'un liderle iletişim durumunu kontrol eder.
        İletişim koptuysa, ROV otomatik olarak lider olur (GNC sistemi tarafından işlenecek).
        ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda (10m içinde) iletişim kopmasını görmezden gel.
        """
        if not self.environment_ref or self.role == 1:  # Lider ise kontrol etme
            return
        
        # Lider ROV'u bul
        lider_rov = None
        for rov in self.environment_ref.rovs:
            if rov.role == 1:
                lider_rov = rov
                break
        
        if lider_rov is None:
            # Lider yok, iletişim yok
            self.lider_ile_iletisim = False
            return
        
        mesafe = distance(self.position, lider_rov.position)
        self_yuzeyde = self.y >= 0
        lider_yuzeyde = lider_rov.y >= 0
        
        # YÜZEY İLETİŞİMİ: Her iki ROV da yüzeydeyse iletişim var
        if self_yuzeyde and lider_yuzeyde:
            self.lider_ile_iletisim = True
        # SU ALTI İLETİŞİMİ: Normal menzil kontrolü
        else:
            iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
            
            # ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda (10m içinde) iletişim kopmasını görmezden gel
            # Bu, çarpışma önleme mekanizmasının neden olduğu geçici iletişim kopmalarını önler
            yakin_mesafe_esigi = 10.0  # 10 metre
            if mesafe < yakin_mesafe_esigi:
                # Çok yakınsa, iletişim var say (geçici kopmaları önle)
                self.lider_ile_iletisim = True
            else:
                self.lider_ile_iletisim = mesafe < iletisim_menzili
    
    def _yaklasma_onleme(self):
        """
        Sensör mesafesine göre ROV'lar ve engellerden uzaklaşma.
        Çarpışmayı önlemek için proaktif kaçınma davranışı.
        """
        if not self.environment_ref:
            return
        
        # Kaçınma mesafesini sensör ayarlarından al
        kacinma_mesafesi = self.sensor_config.get("kacinma_mesafesi", None)
        if kacinma_mesafesi is None:
            # Eğer kacinma_mesafesi yoksa, engel_mesafesi'nin bir kısmını kullan
            engel_mesafesi = self.sensor_config.get("engel_mesafesi", 20.0)
            kacinma_mesafesi = engel_mesafesi * 0.2  # Engel mesafesinin %20'si
        
        uzaklasma_vektoru = Vec3(0, 0, 0)
        
        # Diğer ROV'lardan uzaklaşma
        for diger_rov in self.environment_ref.rovs:
            if diger_rov.id == self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            
            # ÖNEMLİ: ROV'lar birbirine çok yakın olduğunda (2m içinde) kaçınma mekanizmasını devre dışı bırak
            # Bu, ROV'ların birbirini sürekli itmesini önler
            minimum_mesafe = 2.0  # 2 metre - çok yakınsa kaçınma yok
            if mesafe < minimum_mesafe:
                continue  # Çok yakınsa kaçınma yapma
            
            # Kaçınma mesafesi veya daha küçük mesafede uzaklaş
            if mesafe <= kacinma_mesafesi and mesafe > 0:
                # Uzaklaşma yönü (bu ROV'dan diğer ROV'a)
                uzaklasma_yonu = (self.position - diger_rov.position).normalized()
                # Mesafe ne kadar küçükse, o kadar güçlü uzaklaş
                # Ancak gücü daha da yumuşat (çok agresif olmasın)
                uzaklasma_gucu = (kacinma_mesafesi - mesafe) / kacinma_mesafesi
                uzaklasma_gucu *= 0.3  # Gücü %30'a indir (daha yumuşak)
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Engellerden uzaklaşma
        for engel in self.environment_ref.engeller:
            mesafe = distance(self.position, engel.position)
            # Engel boyutunu dikkate al
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            gercek_mesafe = mesafe - engel_yari_cap
            
            # ÖNEMLİ: Engel çok yakınsa (engel yarıçapı + 1m içinde) kaçınma mekanizmasını devre dışı bırak
            # Bu, ROV'ların engellere çok yaklaşmasını önler ama sürekli itmeyi engeller
            minimum_engel_mesafe = engel_yari_cap + 1.0  # Engel yarıçapı + 1 metre
            if gercek_mesafe < minimum_engel_mesafe:
                continue  # Çok yakınsa kaçınma yapma (sadece çarpışma kontrolü yeterli)
            
            # Kaçınma mesafesi veya daha küçük mesafede uzaklaş
            if gercek_mesafe <= kacinma_mesafesi and gercek_mesafe > 0:
                # Uzaklaşma yönü (bu ROV'dan engele)
                uzaklasma_yonu = (self.position - engel.position).normalized()
                # Mesafe ne kadar küçükse, o kadar güçlü uzaklaş
                # Ancak gücü daha da yumuşat (çok agresif olmasın)
                uzaklasma_gucu = (kacinma_mesafesi - gercek_mesafe) / kacinma_mesafesi
                uzaklasma_gucu *= 0.3  # Gücü %30'a indir (daha yumuşak)
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Uzaklaşma vektörünü uygula
        if uzaklasma_vektoru.length() > 0:
            # Normalize et ve güç uygula
            uzaklasma_vektoru = uzaklasma_vektoru.normalized()
            uzaklasma_gucu = min(uzaklasma_vektoru.length(), 1.0)  # Maksimum %100 güç
            
            # Daha yumuşak uzaklaşma için gücü azalt (çarpışma önleme daha yumuşak olsun)
            yumusaklik_carpani = 0.2  # Uzaklaşma gücünü %20'ye indir (daha yumuşak)
            uzaklasma_gucu *= yumusaklik_carpani
            
            # Hız vektörüne ekle (momentum korunumu için)
            uzaklasma_hizi = uzaklasma_vektoru * uzaklasma_gucu * HIZLANMA_CARPANI * time.dt
            self.velocity += uzaklasma_hizi
            
            # Hız limiti (aşırı hızlanmayı önle)
            max_hiz = 50.0
            if self.velocity.length() > max_hiz:
                self.velocity = self.velocity.normalized() * max_hiz
    
    def _carpisma_kontrolu(self):
        """
        Çarpışma kontrolü ve momentum korunumu ile gerçekçi çarpışma.
        """
        if not self.environment_ref:
            return
        
        # ROV kütlesi (basitleştirilmiş)
        rov_kutlesi = 1.0
        
        # Diğer ROV'larla çarpışma
        for diger_rov in self.environment_ref.rovs:
            if diger_rov.id == self.id:
                continue
            
            mesafe = distance(self.position, diger_rov.position)
            min_mesafe = 2.0  # ROV boyutlarına göre minimum mesafe
            
            if mesafe < min_mesafe:
                # Çarpışma tespit edildi
                # Normalize edilmiş çarpışma yönü
                carpisma_yonu = (self.position - diger_rov.position).normalized()
                
                # Göreceli hız
                goreceli_hiz = self.velocity - diger_rov.velocity
                goreceli_hiz_buyuklugu = goreceli_hiz.length()
                
                if goreceli_hiz_buyuklugu > 0.1:
                    # Momentum korunumu (elastik çarpışma)
                    # Basitleştirilmiş: Her iki ROV da aynı kütlede
                    diger_rov_kutlesi = 1.0
                    
                    # Çarpışma sonrası hızlar (momentum korunumu)
                    # v1' = v1 - 2*m2/(m1+m2) * (v1-v2) · n * n
                    # v2' = v2 - 2*m1/(m1+m2) * (v2-v1) · n * n
                    
                    nokta_carpim = goreceli_hiz.dot(carpisma_yonu)
                    
                    if nokta_carpim < 0:  # Birbirine yaklaşıyorlar
                        # Yeni hızlar
                        # Ursina'da Vec3 * float çalışır, float * Vec3 çalışmaz
                        carpan1 = (2 * diger_rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * nokta_carpim
                        self.velocity = self.velocity - carpisma_yonu * carpan1
                        
                        carpan2 = (2 * rov_kutlesi / (rov_kutlesi + diger_rov_kutlesi)) * (-nokta_carpim)
                        diger_rov.velocity = diger_rov.velocity - (-carpisma_yonu) * carpan2
                        
                        # Çarpışma sonrası pozisyonları ayır
                        ayirma_mesafesi = (min_mesafe - mesafe) / 2
                        self.position += carpisma_yonu * ayirma_mesafesi
                        diger_rov.position -= carpisma_yonu * ayirma_mesafesi
        
        # Kayalarla çarpışma
        for engel in self.environment_ref.engeller:
            mesafe = distance(self.position, engel.position)
            # Engel boyutuna göre minimum mesafe
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            min_mesafe = engel_yari_cap + 1.0
            
            if mesafe < min_mesafe:
                # Kaya ile çarpışma
                carpisma_yonu = (self.position - engel.position).normalized()
                
                # Hızı yansıt (kaya sabit, ROV geri seker)
                hiz_buyuklugu = self.velocity.length()
                if hiz_buyuklugu > 0.1:
                    # Yansıma (momentum korunumu - kaya çok ağır, ROV geri seker)
                    nokta_carpim = self.velocity.dot(carpisma_yonu)
                    if nokta_carpim < 0:  # Kayaya doğru gidiyor
                        # Ursina'da Vec3 * float çalışır, float * Vec3 çalışmaz
                        self.velocity = self.velocity - carpisma_yonu * (2 * nokta_carpim)
                        
                        # Pozisyonu ayır
                        ayirma_mesafesi = (min_mesafe - mesafe)
                        self.position += carpisma_yonu * ayirma_mesafesi

class Ortam:
    def __init__(self):
        # --- Ursina Ayarları ---
        self.app = Ursina(
            vsync=False,
            development_mode=False,
            show_ursina_splash=False,
            borderless=False,
            title="FıratROVNet Simülasyonu"
        )
        
        window.fullscreen = False
        window.exit_button.visible = False
        window.fps_counter.enabled = True
        window.size = (1024, 768)
        window.center_on_screen()
        application.run_in_background = True
        window.color = color.rgb(10, 30, 50)  # Arka plan
        
        # Sağ tıklama menüsünü kapat (mouse.right event'lerini yakalamak için)
        try:
            window.context_menu = False
        except:
            pass
        EditorCamera()
        self.editor_camera = EditorCamera()
        self.editor_camera.enabled = False  # Başlangıçta kapalı

        # --- Sahne Nesneleri ---
        self.surface = Entity(
            model='plane',
            scale=(500,1,500),
            color=color.cyan,
            alpha=0.3,
            y=0,
            unlit=True,
            double_sided=True,
            transparent=True
        )

        # Su hacmi parametreleri
        su_hacmi_yuksekligi = 100.0
        su_hacmi_merkez_y = -50.0
        
        self.water_volume = Entity(
            model='cube',
            scale=(500, su_hacmi_yuksekligi, 500),
            color=color.cyan,
            alpha=0.2,
            y=su_hacmi_merkez_y,
            unlit=True,
            double_sided=True,
            transparent=True
        )

        # Deniz tabanı kalınlığı: Su hacmi yüksekliğinin 0.1'i
        seabed_kalinligi = su_hacmi_yuksekligi * 0.1
        # Deniz tabanı alt yüzeyi: Su hacminin altı
        seabed_alt_yuzey = su_hacmi_merkez_y - (su_hacmi_yuksekligi / 2)
        # Deniz tabanı merkez y: Alt yüzeyin üstünde kalınlığın yarısı kadar
        seabed_merkez_y = seabed_alt_yuzey - (seabed_kalinligi / 2)
        
        # Deniz tabanı - Kalın, opak, kum/toprak görünümlü
        self.seabed = Entity(
            model='cube',
            scale=(500, seabed_kalinligi, 500),
            color=color.rgb(139, 90, 43),  # Kahverengi/kum rengi
            y=seabed_merkez_y,
            unlit=True,
            texture='brick',  # Kum/toprak görünümü için
            double_sided=False
        )
        
        # Çimen katmanı kalınlığı: Su hacmi yüksekliğinin 0.25'i
        cimen_kalinligi = su_hacmi_yuksekligi * 0.25
        # Çimen katmanı alt yüzeyi: Deniz tabanının altı
        cimen_alt_yuzey = seabed_merkez_y - (seabed_kalinligi / 2)
        # Çimen katmanı merkez y
        cimen_merkez_y = cimen_alt_yuzey - (cimen_kalinligi / 2)
        
        # Çimen katmanı - Deniz tabanının altında
        self.cimen_katmani = Entity(
            model='cube',
            scale=(500, cimen_kalinligi, 500),
            color=color.rgb(34, 139, 34),  # Çimen yeşili
            y=cimen_merkez_y,
            unlit=True,
            texture='grass',  # Çimen texture'ı
            double_sided=False
        )

        # ROV ve engel listeleri
        self.rovs = []
        self.filo = None  # Filo referansı (main.py'den set edilecek)
        self.engeller = []

        # Konsol verileri
        self.konsol_verileri = {}

    # --- Simülasyon Nesnelerini Oluştur ---
    def sim_olustur(self, n_rovs=3, n_engels=15, havuz_genisligi=200):
        # Engeller
        for _ in range(n_engels):
            x = random.uniform(-200, 200)
            z = random.uniform(-200, 200)
            y = random.uniform(-90, 0)

            s_x = random.uniform(15,40)
            s_y = random.uniform(15,40)
            s_z = random.uniform(-30,30)

            gri = random.randint(80,100)
            kaya_rengi = color.rgb(gri, gri, gri)

            engel = Entity(
                model='icosphere',
                color=kaya_rengi,
                texture='noise',
                scale=(s_x,s_y,s_z),
                position=(x,y,z),
                rotation=(random.randint(0,360), random.randint(0,360), random.randint(0,360)),
                collider='mesh',
                unlit=True
            )
            self.engeller.append(engel)

        # ROV'lar
        for i in range(n_rovs):
            x = random.uniform(-10,10)
            z = random.uniform(-10,10)
            new_rov = ROV(rov_id=i, position=(x,-2,z))  # ROV sınıfın kendi tanımlı olmalı
            new_rov.environment_ref = self
            # Filo referansını ekle (eğer varsa)
            if hasattr(self, 'filo'):
                new_rov.filo_ref = self.filo
            self.rovs.append(new_rov)

        print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {n_engels} Gri Kaya.")

    # --- İnteraktif Shell ---
    def _start_shell(self):
        import time
        time.sleep(1)
        print("\n" + "="*60)
        print("🚀 FIRAT ROVNET CANLI KONSOL")
        print("Çıkmak için Ctrl+D veya 'exit()' yazın.")
        print("="*60 + "\n")

        local_vars = {
            'rovs': self.rovs,
            'engeller': self.engeller,
            'app': self,
            'ursina': sys.modules['ursina'],
            'cfg': cfg
        }
        if hasattr(self, 'konsol_verileri'):
            local_vars.update(self.konsol_verileri)

        try:
            code.interact(local=dict(globals(), **local_vars))
        except SystemExit:
            pass
        except Exception as e:
            print(f"Konsol Hatası: {e}")
        finally:
            print("Konsol kapatılıyor...")
            import os
            os.system('stty sane')
            os._exit(0)

    # --- Update Fonksiyonunu Set Et ---
    def set_update_function(self, func):
        self.app.update = func

    # --- Konsola Veri Ekle ---
    def konsola_ekle(self, isim, nesne):
        self.konsol_verileri[isim] = nesne

    # --- Veri Toplama Fonksiyonu (GAT Girdisi) ---
    def simden_veriye(self):
        """
        Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)
        
        Returns:
            MiniData: GAT modeli için hazırlanmış veri yapısı (x, edge_index)
        """
        rovs = self.rovs
        engeller = self.engeller
        n = len(rovs)
        x = torch.zeros((n, 7), dtype=torch.float)
        positions = [r.position for r in rovs]
        sources, targets = [], []

        L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}

        for i in range(n):
            code = 0
            if i != 0 and distance(positions[i], positions[0]) > L['LEADER']: 
                code = 5
            dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
            if dists and min(dists) > L['DISCONNECT']: 
                code = 3
            
            min_engel = 999
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6 
                if d < min_engel: 
                    min_engel = d
            if min_engel < L['OBSTACLE']: 
                code = 1
            
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['COLLISION']:
                    code = 2
                    break
            
            x[i][0] = code / 5.0
            x[i][1] = rovs[i].battery  # Batarya artık 0-1 arası, bölmeye gerek yok
            x[i][2] = 0.9
            x[i][3] = abs(rovs[i].y) / 100.0
            x[i][4] = rovs[i].velocity.x
            x[i][5] = rovs[i].velocity.z
            x[i][6] = rovs[i].role

            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        class MiniData:
            def __init__(self, x, edge_index): 
                self.x, self.edge_index = x, edge_index
        return MiniData(x, edge_index)

    # --- Main Run Fonksiyonu ---
    def run(self, interaktif=False):
        if interaktif:
            t = threading.Thread(target=self._start_shell)
            t.daemon = True
            t.start()
        self.app.run()
