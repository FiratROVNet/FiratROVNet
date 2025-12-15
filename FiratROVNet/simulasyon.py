from ursina import *
import numpy as np
import random
import threading
import code
import sys
import torch
import math
    
from .config import cfg # <-- BU SATIRI EKLE

  

# --- FİZİK SABİTLERİ ---
SURTUNME_KATSAYISI = 0.95
HIZLANMA_CARPANI = 0.5
KALDIRMA_KUVVETI = 2.0
BATARYA_SOMURME_KATSAYISI = 0.01  # Batarya tüketim katsayısı (küçük değer, batarya yavaş bitsin)



class ROV(Entity):
    def __init__(self, rov_id, model_yolu=None, **kwargs):
        super().__init__()
        
        # 3D Model Desteği
        if model_yolu:
            # Model yolu verilmişse kullan (zaten _rov_modeli_bul tarafından kontrol edilmiş)
            if os.path.exists(model_yolu):
                self.model = model_yolu
            else:
                # Model bulunamadı, varsayılan cube kullan
                self.model = 'cube'
        else:
            # Varsayılan: cube modeli
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
        self.battery = 100.0
        self.role = 0
        self.batarya_bitti = False  # Batarya bitme durumu
        self.calistirilan_guc = 0.0  # Çalıştırılan güç (batarya tüketimi için) 
        
        self.sensor_config = {
            "engel_mesafesi": 20.0,
            "iletisim_menzili": 35.0,
            "min_pil_uyarisi": 10.0
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
        self.iletisim_rovlari = {}  # {rov_id: {'mesafe': float, 'cizgi': Entity}} 

    def update(self):
        # Batarya tüketimi (gerçekçi fizik)
        if self.battery > 0:
            # Çalıştırılan güç hesapla (hız ve hareket durumuna göre)
            mevcut_guc = abs(self.velocity.length()) / 100.0  # 0.0-1.0 arası normalize
            if mevcut_guc > 0.01:  # Hareket varsa
                self.calistirilan_guc = mevcut_guc
                # Batarya tüketimi: batarya = batarya - gecen_sure * rov_calistirilan_guc * somurme_katsayisi
                self.battery -= time.dt * self.calistirilan_guc * BATARYA_SOMURME_KATSAYISI
                self.battery = max(0.0, self.battery)  # Negatif olamaz
            else:
                self.calistirilan_guc = 0.0  # Duruyorsa güç tüketimi yok
        
        # Batarya bitti mi kontrol et
        if self.battery <= 0 and not self.batarya_bitti:
            self.batarya_bitti = True
            # Manuel kontrolü aç (sürüden ayrıl)
            if self.environment_ref:
                # GNC sistemine eriş (eğer varsa)
                for gnc in getattr(self.environment_ref, 'gnc_sistemleri', []):
                    if hasattr(gnc, 'rov') and gnc.rov.id == self.id:
                        gnc.manuel_kontrol = True
                        break
            # Yüzeye çık
            self.velocity = Vec3(0, 0, 0)
            # Renk değiştir (batarya bitti rengi)
            self.color = color.rgb(100, 100, 100)  # Gri (batarya bitti)
            print(f"[ROV-{self.id}] Batarya bitti! Yuzeye cikiyor...")
        
        # Batarya bitmişse hareket ettirme
        if self.batarya_bitti:
            # Sadece yüzeye çık
            if self.y < 0:
                self.velocity.y = 2.0  # Yüzeye çık
            else:
                self.velocity = Vec3(0, 0, 0)  # Yüzeyde dur
            # Manuel hareketi engelle
            if self.manuel_hareket['yon'] is not None:
                self.manuel_hareket['yon'] = None
                self.manuel_hareket['guc'] = 0.0
            return  # Batarya bitmişse diğer işlemleri yapma
        
        # Manuel hareket kontrolü (sürekli hareket için - gerçekçi fizik ile)
        if self.manuel_hareket['yon'] is not None and self.manuel_hareket['guc'] > 0:
            if self.manuel_hareket['yon'] == 'dur':
                self.velocity *= 0.9  # Yavaşça dur (momentum korunumu)
                if self.velocity.length() < 0.1:
                    self.velocity = Vec3(0, 0, 0)
                    self.manuel_hareket['yon'] = None
                    self.manuel_hareket['guc'] = 0.0
            else:
                # Sürekli hareket: Gerçekçi fizik ile
                yon = self.manuel_hareket['yon']
                guc = self.manuel_hareket['guc']
                
                # Yönü vektöre çevir
                hareket_vektoru = Vec3(0, 0, 0)
                if yon == 'ileri': hareket_vektoru.z = 1.0
                elif yon == 'geri': hareket_vektoru.z = -1.0
                elif yon == 'sag': hareket_vektoru.x = 1.0
                elif yon == 'sol': hareket_vektoru.x = -1.0
                elif yon == 'cik': hareket_vektoru.y = 1.0
                elif yon == 'bat' and self.role != 1: hareket_vektoru.y = -1.0
                
                # Gerçekçi fizik: Momentum korunumu ve su direnci
                max_guc = 100.0 * guc  # Hız limiti
                
                # Su direnci faktörü (derinlik arttıkça direnç artar)
                derinlik_faktoru = 1.0 - (abs(self.y) / 100.0) * 0.1
                derinlik_faktoru = max(0.9, min(1.0, derinlik_faktoru))
                
                # Momentum korunumu: Mevcut hızı dikkate al
                mevcut_hiz = self.velocity.length()
                if mevcut_hiz > 0:
                    # Yeni hız = eski hız + ivme (momentum korunumu)
                    ivme = hareket_vektoru.normalized() * max_guc * derinlik_faktoru * time.dt * 0.5
                    self.velocity += ivme
                else:
                    # Sıfırdan başlıyorsa direkt ivme uygula
                    ivme = hareket_vektoru.normalized() * max_guc * derinlik_faktoru * time.dt * 0.5
                    self.velocity += ivme
                
                # Hız limiti kontrolü
                if self.velocity.length() > max_guc:
                    self.velocity = self.velocity.normalized() * max_guc
                
                # Lider ROV için aşağı hızı engelle
                if self.role == 1 and self.velocity.y < 0:
                    self.velocity.y = 0
        
        # Havuz sınır kontrolü
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
        
        # Engel tespiti (her zaman çalışır, manuel kontrol olsun olmasın)
        if self.environment_ref:
            self._engel_tespiti()
        
        # Sonar iletişim tespiti (ROV'lar arası kesikli çizgi)
        if self.environment_ref:
            self._sonar_iletisim()
        
        # Fizik
        self.position += self.velocity * time.dt
        self.velocity *= SURTUNME_KATSAYISI
        
        # Çarpışma kontrolü
        if self.environment_ref:
            self._carpisma_kontrolu()
        
        if self.role == 1: # Lider - Su yüzeyinde kalmalı, batırılamaz
            # Lider her zaman su yüzeyine çıkar
            if self.y < 0:
                self.velocity.y += KALDIRMA_KUVVETI * 2.0 * time.dt  # Daha güçlü kaldırma
                if self.y > -0.3: self.velocity.y *= 0.3
            # Su yüzeyi limitleri
            if self.y < -1.0: 
                self.y = -1.0  # Su yüzeyine zorla çıkar
                self.velocity.y = max(0, self.velocity.y)  # Aşağı hızı sıfırla
            if self.y > 0.5: 
                self.y = 0.5
                self.velocity.y = 0
            # Lider asla batırılamaz - aşağı hızı engelle
            if self.velocity.y < 0:
                self.velocity.y = 0
        else: # Takipçi - Serbest hareket
            if self.y > 0: 
                self.y = 0
                self.velocity.y = 0
            if self.y < -100: 
                self.y = -100
                self.velocity.y = 0

        if self.velocity.length() > 0.01: 
            self.battery -= 0.01 * time.dt

    def move(self, komut, guc=1.0):
        """
        ROV'a hareket komutu verir.
        
        Args:
            komut: Hareket yönü ('ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur')
            guc: Motor gücü (0.0-1.0, varsayılan: 1.0)
        
        Not: Eğer guc > 0 ise, manuel hareket modu aktif olur ve sürekli hareket eder.
        """
        # Güç değerini sınırla
        guc = max(0.0, min(1.0, guc))
        
        # Manuel hareket modunu ayarla (sürekli hareket için)
        if guc > 0 and komut != 'dur':
            self.manuel_hareket['yon'] = komut
            self.manuel_hareket['guc'] = guc
        elif komut == 'dur' or guc == 0:
            self.manuel_hareket['yon'] = None
            self.manuel_hareket['guc'] = 0.0
            self.velocity = Vec3(0, 0, 0)
            return
        
        # Anlık hareket uygula
        thrust = guc * HIZLANMA_CARPANI * time.dt
        if self.battery <= 0 or self.batarya_bitti: return  # Batarya bitmişse hareket ettirme

        if komut == "ileri":  self.velocity.z += thrust
        elif komut == "geri": self.velocity.z -= thrust
        elif komut == "sag":  self.velocity.x += thrust
        elif komut == "sol":  self.velocity.x -= thrust
        elif komut == "cik":  self.velocity.y += thrust 
        elif komut == "bat":  
            # Lider batırılamaz
            if self.role == 1: 
                pass  # Lider için bat komutu işe yaramaz
            else: 
                self.velocity.y -= thrust

    def set(self, ayar_adi, deger):
        if ayar_adi == "rol":
            eski_rol = self.role
            self.role = int(deger)
            if self.role == 1:
                self.color = color.red
                self.label.text = f"LIDER-{self.id}"
                # Lider olduğunda su yüzeyine çıkar
                if self.y < 0:
                    self.y = 0
                    self.velocity.y = 0
                print(f"✅ ROV-{self.id} artık LİDER (Su yüzeyinde).")
            else:
                self.color = color.orange
                self.label.text = f"ROV-{self.id}"
                # Takipçi olduğunda artık batırılabilir
                print(f"✅ ROV-{self.id} artık TAKİPÇİ (Batırılabilir).")
        elif ayar_adi == "renk":
            # Renk ayarlama
            if isinstance(deger, (tuple, list)) and len(deger) >= 3:
                self.color = color.rgb(int(deger[0]), int(deger[1]), int(deger[2]))
            elif isinstance(deger, str):
                # Renk ismi ile
                renk_dict = {
                    'kirmizi': color.red, 'mavi': color.blue, 'yesil': color.green,
                    'sari': color.yellow, 'turuncu': color.orange, 'mor': color.magenta,
                    'beyaz': color.white, 'siyah': color.black
                }
                self.color = renk_dict.get(deger.lower(), color.white)
        elif ayar_adi in self.sensor_config: 
            self.sensor_config[ayar_adi] = deger

    def get(self, veri_tipi):
        if veri_tipi == "gps": return np.array([self.x, self.y, self.z])
        elif veri_tipi == "hiz": return np.array([self.velocity.x, self.velocity.y, self.velocity.z])
        elif veri_tipi == "batarya": return self.battery
        elif veri_tipi == "rol": return self.role
        elif veri_tipi == "renk": return self.color
        elif veri_tipi == "sensör" or veri_tipi == "sensor":
            return self.sensor_config.copy()
        elif veri_tipi == "engel_mesafesi": return self.sensor_config.get("engel_mesafesi")
        elif veri_tipi == "iletisim_menzili": return self.sensor_config.get("iletisim_menzili")
        elif veri_tipi == "min_pil_uyarisi": return self.sensor_config.get("min_pil_uyarisi")
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
        """
        if not self.environment_ref:
            return
        
        # İletişim menzili
        iletisim_menzili = self.sensor_config.get("iletisim_menzili", 35.0)
        
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
            
            # İletişim menzili içindeyse
            if mesafe < iletisim_menzili:
                aktif_iletisim_rovlari[diger_rov.id] = {
                    'rov': diger_rov,
                    'mesafe': mesafe
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
            
            # Eğer zaten iletişim varsa güncelle, yoksa yeni çiz
            if rov_id in self.iletisim_rovlari:
                # Mevcut çizgiyi güncelle
                if self.iletisim_rovlari[rov_id].get('cizgi'):
                    destroy(self.iletisim_rovlari[rov_id]['cizgi'])
            
            # Yeni çizgi çiz
            cizgi = self._rov_arasi_cizgi_ciz(diger_rov, mesafe)
            
            # İletişim bilgisini güncelle
            self.iletisim_rovlari[rov_id] = {
                'rov': diger_rov,
                'mesafe': mesafe,
                'cizgi': cizgi
            }
    
    def _rov_arasi_cizgi_ciz(self, diger_rov, mesafe):
        """
        İki ROV arasında kesikli çizgi çizer (sonar iletişimi).
        
        Args:
            diger_rov: İletişim kurulan diğer ROV
            mesafe: İki ROV arasındaki mesafe
        
        Returns:
            Entity: Çizgi entity'si
        """
        # Çizgi rengi: mesafeye göre (yakın = mavi, uzak = cyan)
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
        self.engeller = []
        
        # Hedef nokta (varsayılan)
        self.hedef_nokta = Vec3(40, 0, 60)
        
        # Havuz genişliği (varsayılan)
        self.havuz_genisligi = 200.0

        # Konsol verileri
        self.konsol_verileri = {}
        
        # Hedef nokta görsel işareti
        self.hedef_isareti = None
        
        # AI ve GNC referansları (main.py'den set edilecek)
        self.beyin = None
        self.filo = None

    # --- Simülasyon Nesnelerini Oluştur ---
    def sim_olustur(self, n_rovs=3, n_engels=15, hedef_nokta=None, havuz_genisligi=200, rov_model_yolu=None):
        """
        Simülasyon nesnelerini oluşturur.
        
        Args:
            n_rovs: ROV sayısı
            n_engels: Engel sayısı
            hedef_nokta: Hedef nokta (Vec3)
            havuz_genisligi: Havuz genişliği
            rov_model_yolu: 3D model dosya yolu (Models-3D klasöründen yüklenecek)
        
        Args:
            n_rovs: ROV sayısı
            n_engels: Engel sayısı
            hedef_nokta: Hedef nokta (Vec3) - engeller bu noktadan uzak olur
            havuz_genisligi: Havuz genişliği (varsayılan: 200)
        """
        # Havuz genişliğini kaydet
        self.havuz_genisligi = havuz_genisligi
        
        # Hedef noktayı ayarla
        if hedef_nokta is None:
            self.hedef_nokta = Vec3(40, 0, 60)
        else:
            self.hedef_nokta = hedef_nokta
        
        # Hedef nokta görsel işareti oluştur
        self._hedef_isareti_olustur()
        
        # Havuz sınırları
        havuz_yari_genislik = havuz_genisligi / 2
        hedef_guvenlik_mesafesi = 30.0  # Hedeften minimum mesafe
        
        # Engeller - Tüm havuza yayılmış, hedeften uzak
        engel_deneme_sayisi = 0
        max_deneme = n_engels * 10  # Maksimum deneme sayısı
        
        while len(self.engeller) < n_engels and engel_deneme_sayisi < max_deneme:
            engel_deneme_sayisi += 1
            
            # Tüm havuza geniş şekilde yayılmış random pozisyon (daha geniş alan)
            x = random.uniform(-havuz_yari_genislik * 0.9, havuz_yari_genislik * 0.9)
            z = random.uniform(-havuz_yari_genislik * 0.9, havuz_yari_genislik * 0.9)
            y = random.uniform(-90, -10)
            
            # Hedef noktadan uzaklık kontrolü
            engel_pos = Vec3(x, y, z)
            if distance(engel_pos, self.hedef_nokta) < hedef_guvenlik_mesafesi:
                continue  # Hedefe çok yakın, tekrar dene
            
            # Boyutlar
            s_x = random.uniform(4, 12)
            s_y = random.uniform(4, 12)
            s_z = random.uniform(4, 12)
            
            # Geniş gri aralığında kaya renkleri (daha geniş spektrum)
            # 40-200 arası gri tonları
            gri_tonu = random.randint(40, 200)
            # Biraz varyasyon ekle (benek efekti için)
            r_varyasyon = random.randint(-15, 15)
            g_varyasyon = random.randint(-15, 15)
            b_varyasyon = random.randint(-15, 15)
            
            kaya_rengi = color.rgb(
                max(30, min(220, gri_tonu + r_varyasyon)),
                max(30, min(220, gri_tonu + g_varyasyon)),
                max(30, min(220, gri_tonu + b_varyasyon))
            )
            
            engel = Entity(
                model='icosphere',
                color=kaya_rengi,
                texture='noise',  # Benek efekti için noise texture
                scale=(s_x, s_y, s_z),
                position=(x, y, z),
                rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
                collider='mesh',
                unlit=True
            )
            self.engeller.append(engel)
        
        # ROV'lar - Engellerden ve hedeften uzak oluştur
        rov_guvenlik_mesafesi = 15.0  # Engellerden minimum mesafe
        rov_deneme_sayisi = 0
        max_rov_deneme = n_rovs * 20
        
        for i in range(n_rovs):
            gecerli_pozisyon = False
            deneme = 0
            
            while not gecerli_pozisyon and deneme < max_rov_deneme:
                deneme += 1
                x = random.uniform(-havuz_yari_genislik * 0.3, havuz_yari_genislik * 0.3)
                z = random.uniform(-havuz_yari_genislik * 0.3, havuz_yari_genislik * 0.3)
                y = -2  # Su yüzeyine yakın
                
                rov_pos = Vec3(x, y, z)
                gecerli_pozisyon = True
                
                # Hedeften uzak mı?
                if distance(rov_pos, self.hedef_nokta) < hedef_guvenlik_mesafesi:
                    gecerli_pozisyon = False
                    continue
                
                # Engellerden uzak mı?
                for engel in self.engeller:
                    if distance(rov_pos, engel.position) < rov_guvenlik_mesafesi:
                        gecerli_pozisyon = False
                        break
                
                if gecerli_pozisyon:
                    # Model yolunu otomatik bul (eğer verilmişse)
                    kullanilacak_model = self._rov_modeli_bul(rov_model_yolu)
                    # None ise varsayılan cube kullanılacak (ROV.__init__ içinde)
                    new_rov = ROV(rov_id=i, position=(x, y, z), model_yolu=kullanilacak_model)
                    new_rov.environment_ref = self
                    self.rovs.append(new_rov)
                    break
            
            # Eğer geçerli pozisyon bulunamazsa varsayılan pozisyon
            if not gecerli_pozisyon:
                x = -20 + (i * 10)
                z = -20 + (i * 10)
                kullanilacak_model = self._rov_modeli_bul(rov_model_yolu)
                # None ise varsayılan cube kullanılacak (ROV.__init__ içinde)
                new_rov = ROV(rov_id=i, position=(x, -2, z), model_yolu=kullanilacak_model)
                new_rov.environment_ref = self
                self.rovs.append(new_rov)
        
        # Model yükleme durumunu bildir (sadece bir kez)
        if rov_model_yolu:
            models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Models-3D')
            full_path = os.path.join(models_dir, rov_model_yolu)
            full_path = os.path.normpath(full_path)
            if os.path.exists(full_path) and os.path.isfile(full_path):
                print(f"[Ortam] ROV modeli yuklendi: {rov_model_yolu}")
            else:
                print(f"[Ortam] ROV modeli bulunamadi: {rov_model_yolu}, varsayilan 'cube' kullaniliyor")
        
        print(f"🌊 Simülasyon Hazır: {n_rovs} ROV, {len(self.engeller)} Kaya, Hedef: {self.hedef_nokta}")
    
    def _rov_modeli_bul(self, model_yolu):
        """
        ROV model dosyasını bulur.
        
        Args:
            model_yolu: Model dosya yolu (None, dosya adı veya tam yol)
                - None: Otomatik arama yapılır (rov.obj, rov.glb, vb.)
                - "rov.obj": Models-3D klasöründen aranır
                - "submarine/model.obj": Models-3D/submarine/model.obj aranır
                - Tam yol: Verilen yol kullanılır
        
        Returns:
            str: Model yolu veya None (varsayılan cube kullanılır)
        """
        models_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Models-3D')
        
        if model_yolu is None:
            # Varsayılan: Models-3D klasöründen otomatik arama
            # Yaygın uzantıları dene: obj, glb, gltf, fbx, dae
            yaygin_uzantilar = ['obj', 'glb', 'gltf', 'fbx', 'dae']
            for uzanti in yaygin_uzantilar:
                model_adi = f'rov.{uzanti}'
                full_path = os.path.join(models_dir, model_adi)
                if os.path.exists(full_path):
                    print(f"[Ortam] ROV modeli bulundu: {model_adi}")
                    return full_path
            return None  # Model bulunamadı, varsayılan cube kullanılacak
        else:
            # Model yolu verilmişse
            # Önce tam yol olarak dene (mutlak yol)
            if os.path.isabs(model_yolu):
                if os.path.exists(model_yolu):
                    return model_yolu
            else:
                # Göreceli yol: Models-3D klasöründen dene (alt klasörler dahil)
                full_path = os.path.join(models_dir, model_yolu)
                # Normalize path (.. ve . işlemlerini çöz)
                full_path = os.path.normpath(os.path.abspath(full_path))
                if os.path.exists(full_path) and os.path.isfile(full_path):
                    return full_path
                
                # Eğer sadece uzantı verilmişse (örn: "obj"), "rov.obj" olarak dene
                model_basename = os.path.basename(model_yolu)
                
                # Uzantıyı al (nokta ile veya noktasız)
                if '.' in model_basename:
                    uzanti = model_basename.split('.')[-1]
                else:
                    uzanti = model_basename
                
                # "rov." ile başlamıyorsa ve sadece uzantıysa, "rov." ekle
                if not model_basename.startswith('rov.') and len(model_basename.split('.')) == 1:
                    rov_model_adi = f'rov.{uzanti}'
                    full_path = os.path.join(models_dir, rov_model_adi)
                    full_path = os.path.normpath(full_path)
                    if os.path.exists(full_path):
                        return full_path
            
            # Model bulunamadı, sessizce None döndür (varsayılan cube kullanılacak)
            return None
    
    def _hedef_isareti_olustur(self):
        """
        Hedef noktayı görsel işaret ile gösterir.
        """
        # Eski işareti kaldır
        if self.hedef_isareti:
            destroy(self.hedef_isareti)
        
        # Hedef nokta işareti oluştur (3D ok veya işaret)
        # Ana işaret (ok)
        self.hedef_isareti = Entity(
            model='cube',
            position=self.hedef_nokta,
            scale=(2, 0.2, 2),
            color=color.green,
            unlit=True
        )
        
        # Üstte dönen ok işareti
        ok_isareti = Entity(
            model='cube',
            position=self.hedef_nokta + Vec3(0, 3, 0),
            scale=(0.5, 2, 0.5),
            color=color.yellow,
            parent=self.hedef_isareti,
            unlit=True
        )
        
        # Altında ışık halkası
        halka = Entity(
            model='circle',
            position=self.hedef_nokta + Vec3(0, 0.1, 0),
            scale=(5, 1, 5),
            color=color.rgb(0, 255, 0),
            alpha=0.4,  # Yarı saydam
            rotation_x=90,
            unlit=True,
            double_sided=True,
            transparent=True
        )
        
        # Animasyon için referans
        self.hedef_isareti.ok = ok_isareti
        self.hedef_isareti.halka = halka
    
    def _hedef_isareti_guncelle(self):
        """
        Hedef işaretini animasyonlu olarak günceller (döndürme, parıldama).
        """
        if self.hedef_isareti:
            # Ok işaretini döndür
            if hasattr(self.hedef_isareti, 'ok'):
                self.hedef_isareti.ok.rotation_y += time.dt * 90  # Saniyede 90 derece
            
            # Halkayı parıldat (alpha değişimi)
            if hasattr(self.hedef_isareti, 'halka'):
                # Sinüs dalgası ile parıldama (0.3-0.7 arası)
                alpha = 0.5 + 0.2 * math.sin(time.time() * 2)
                self.hedef_isareti.halka.alpha = alpha

    # --- GAT Veri Dönüşüm Fonksiyonu ---
    def simden_veriye(self, limitler=None):
        """
        Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)
        
        Args:
            limitler (dict, optional): Mesafe limitleri. Varsayılan değerler:
                - 'LEADER': 60.0 (Liderden uzaklık limiti)
                - 'DISCONNECT': 35.0 (Bağlantı kopma limiti)
                - 'OBSTACLE': 20.0 (Engel tespit limiti)
                - 'COLLISION': 8.0 (Çarpışma tespit limiti)
        
        Returns:
            MiniData: x (özellik matrisi) ve edge_index (graf bağlantıları) içeren nesne
        """
        # Varsayılan limitler
        if limitler is None:
            limitler = {
                'LEADER': 60.0,
                'DISCONNECT': 35.0,
                'OBSTACLE': 20.0,
                'COLLISION': 8.0
            }
        
        rovs = self.rovs
        engeller = self.engeller
        n = len(rovs)
        x = torch.zeros((n, 7), dtype=torch.float)
        positions = [r.position for r in rovs]
        sources, targets = [], []

        for i in range(n):
            code = 0
            # Liderden uzaklık kontrolü
            if i != 0 and distance(positions[i], positions[0]) > limitler['LEADER']: 
                code = 5
            
            # Bağlantı kopma kontrolü
            dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
            if dists and min(dists) > limitler['DISCONNECT']: 
                code = 3
            
            # Engel tespiti
            min_engel = 999
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6 
                if d < min_engel: 
                    min_engel = d
            if min_engel < limitler['OBSTACLE']: 
                code = 1
            
            # Çarpışma kontrolü
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < limitler['COLLISION']:
                    code = 2
                    break
            
            # Özellik vektörü oluştur
            x[i][0] = code / 5.0
            x[i][1] = rovs[i].battery / 100.0
            x[i][2] = 0.9  # SNR (sabit)
            x[i][3] = abs(rovs[i].y) / 100.0
            x[i][4] = rovs[i].velocity.x
            x[i][5] = rovs[i].velocity.z
            x[i][6] = rovs[i].role

            # Graf bağlantıları (iletişim menzili içindeki ROV'lar)
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < limitler['DISCONNECT']:
                    sources.append(i)
                    targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        
        # MiniData sınıfı (PyG Data yapısını taklit eder)
        class MiniData:
            def __init__(self, x, edge_index): 
                self.x, self.edge_index = x, edge_index
        
        return MiniData(x, edge_index)

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

    # --- ROV Görsel Güncellemeleri ---
    def guncelle_rov_gorselleri(self, tahminler, ai_aktif=True):
        """
        ROV'ların renk ve label'larını GAT kodlarına göre günceller.
        
        Args:
            tahminler: GAT kodları listesi (her ROV için)
            ai_aktif: AI aktif mi (varsayılan: True)
        """
        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
        
        # Her ROV için GAT koduna göre renk belirleme (manuel kontrol olsun olmasın)
        for i, gat_kodu in enumerate(tahminler):
            if i >= len(self.rovs):
                continue
                
            rov = self.rovs[i]
            
            # Batarya bitmişse özel renk (gri)
            if rov.batarya_bitti:
                rov.color = color.rgb(100, 100, 100)  # Gri (batarya bitti)
            # Lider ROV her zaman kırmızı (batarya bitmemişse)
            elif rov.role == 1: 
                rov.color = color.red
            else: 
                # GAT koduna göre renk (manuel kontrol olsun olmasın)
                rov.color = kod_renkleri.get(gat_kodu, color.white)
            
            # Sensör bazlı engel tespiti (GAT olmasa bile, batarya bitmemişse)
            if not rov.batarya_bitti and rov.tespit_edilen_engel is not None:
                # Engel tespit edildi, renk kırmızıya yakın olsun
                if gat_kodu == 0:  # GAT engel tespit etmediyse ama sensör tespit ettiyse
                    rov.color = color.rgb(255, 100, 0)  # Turuncu-kırmızı
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            # Engel mesafesi bilgisi ekle
            if rov.tespit_edilen_engel is not None:
                mesafe_bilgisi = f"\n{rov.engel_mesafesi:.1f}m"
            else:
                mesafe_bilgisi = ""
            # Batarya bilgisi ekle (emoji yerine metin kullan)
            batarya_bilgisi = f"\nBAT:{rov.battery:.0f}%"
            if rov.batarya_bitti:
                batarya_bilgisi = "\nBAT:BITTI"
            rov.label.text = f"R{i}\n{durum_txts[gat_kodu]}{mesafe_bilgisi}{batarya_bilgisi}{ek}"
    
    # --- Ana Update Fonksiyonu ---
    def guncelle(self):
        """
        Simülasyonun ana update fonksiyonu.
        GAT analizi, görsel güncellemeler ve GNC güncellemelerini yapar.
        """
        try:
            # Simülasyondan GAT verisi al
            veri = self.simden_veriye()
            
            # AI analizi
            ai_aktif = getattr(cfg, 'ai_aktif', True)
            if ai_aktif and self.beyin:
                try: 
                    tahminler, _, _ = self.beyin.analiz_et(veri)
                except: 
                    tahminler = np.zeros(len(self.rovs), dtype=int)
            else:
                tahminler = np.zeros(len(self.rovs), dtype=int)
            
            # ROV görsel güncellemeleri
            self.guncelle_rov_gorselleri(tahminler, ai_aktif)
            
            # GNC güncellemeleri
            if self.filo:
                if len(self.filo.sistemler) > 0:
                    self.filo.guncelle_hepsi(tahminler)
                else:
                    # GNC sistemleri henüz eklenmemiş
                    pass
            else:
                # Filo henüz set edilmemiş, ROV'lar hareket etmeyecek
                # İlk birkaç frame'de bu normal olabilir
                pass
                
        except Exception as e: 
            # Hata ayıklama için (geliştirme sırasında)
            # print(f"[HATA] guncelle(): {e}")
            # import traceback
            # traceback.print_exc()
            pass  # Sessizce devam et

    # --- Update Fonksiyonunu Set Et ---
    def set_update_function(self, func=None):
        """
        Update fonksiyonunu ayarlar.
        
        Args:
            func: Özel update fonksiyonu (None ise varsayılan guncelle() kullanılır)
        """
        if func is None:
            # Varsayılan update fonksiyonu
            def wrapped_update():
                # Hedef işaretini güncelle (animasyon)
                if self.hedef_isareti:
                    self._hedef_isareti_guncelle()
                # Ana güncelleme
                self.guncelle()
            self.app.update = wrapped_update
        else:
            # Özel update fonksiyonu
            def wrapped_update():
                # Hedef işaretini güncelle (animasyon)
                if self.hedef_isareti:
                    self._hedef_isareti_guncelle()
                # Kullanıcı update fonksiyonunu çağır
                func()
            self.app.update = wrapped_update

    # --- Konsola Veri Ekle ---
    def konsola_ekle(self, isim, nesne):
        self.konsol_verileri[isim] = nesne

    # --- Main Run Fonksiyonu ---
    def run(self, interaktif=False):
        if interaktif:
            t = threading.Thread(target=self._start_shell)
            t.daemon = True
            t.start()
        self.app.run()
