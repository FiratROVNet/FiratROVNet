import numpy as np
from ursina import Vec3, time, distance
from .config import cfg
from .iletisim import AkustikModem
import math
import random

# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo:
    def __init__(self):
        self.sistemler = [] 
        self.suru_lider_secildi = {}  # {grup_id: True} - Hangi gruplar için lider seçildi
        self.asil_hedef = None  # Asıl hedef (orijinal liderin hedefi)
        self.orijinal_lider_id = 0  # Orijinal lider ID
        self.lider_degisim_sayaci = {}  # {rov_id: sayac} - Lider değişim sayacı (sonsuz döngü önleme)
        self.korunan_rovlar = set()  # {rov_id} - Sonsuz döngü önleme tarafından korunan ROV'lar (takipçi yapılmaz)

    def ekle(self, gnc_objesi):
        self.sistemler.append(gnc_objesi)

    def rehber_dagit(self, modem_rehberi):
        if self.sistemler:
            for sistem in self.sistemler:
                if isinstance(sistem, LiderGNC):
                    sistem.rehber_guncelle(modem_rehberi)

    def otomatik_kurulum(self, rovs, lider_id=0, modem_ayarlari=None, baslangic_hedefleri=None, sensor_ayarlari=None):
        """
        ROV filo sistemini otomatik olarak kurar ve yapılandırır.
        
        Bu fonksiyon tüm ROV'lar için modem, GNC sistemi, sensör ayarları ve başlangıç hedeflerini
        otomatik olarak oluşturur. Manuel kurulum ihtiyacını ortadan kaldırır.
        
        Args:
            rovs: ROV entity listesi (Ortam.rovs)
            lider_id (int): Lider ROV'un ID'si (varsayılan: 0)
            modem_ayarlari (dict, optional): Modem parametreleri. Örnek:
                {
                    'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.1, 'gecikme': 0.5},
                    'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1, 'gecikme': 0.5}
                }
            baslangic_hedefleri (dict, optional): ROV ID'lerine göre başlangıç hedefleri. Örnek:
                {
                    0: (40, 0, 60),    # Lider: (x, y, z)
                    1: (35, -10, 50),  # Takipçi 1
                    2: (40, -10, 50),  # Takipçi 2
                    3: (45, -10, 50)   # Takipçi 3
                }
            sensor_ayarlari (dict, optional): Sensör ayarları. Üç format desteklenir:
                # Format 1: Tüm ROV'lar için ortak ayarlar
                {
                    'engel_mesafesi': 25.0,
                    'iletisim_menzili': 40.0,
                    'min_pil_uyarisi': 15.0
                }
                # Format 2: Lider ve takipçi için ayrı ayarlar
                {
                    'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
                    'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
                }
                # Format 3: Her ROV için özel ayarlar (ROV ID ile)
                {
                    0: {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0},  # Lider
                    1: {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0},  # Takipçi 1
                    2: {'engel_mesafesi': 20.0, 'iletisim_menzili': 35.0}   # Takipçi 2
                }
        
        Returns:
            dict: Tüm modemlerin rehberi (rehber_dagit için kullanılabilir)
        
        Örnekler:
            # Basit kullanım (varsayılan ayarlar)
            filo = Filo()
            tum_modemler = filo.otomatik_kurulum(rovs=app.rovs)
            
            # Özel başlangıç hedefleri ile
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                baslangic_hedefleri={
                    0: (40, 0, 60),    # Lider
                    1: (35, -10, 50),  # Takipçi 1
                    2: (40, -10, 50),  # Takipçi 2
                    3: (45, -10, 50)   # Takipçi 3
                }
            )
            
            # Özel modem ayarları ile
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                modem_ayarlari={
                    'lider': {'gurultu_orani': 0.03, 'kayip_orani': 0.05, 'gecikme': 0.3},
                    'takipci': {'gurultu_orani': 0.15, 'kayip_orani': 0.2, 'gecikme': 0.6}
                }
            )
            
            # Tüm parametrelerle tam kontrol
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                lider_id=0,
                modem_ayarlari={
                    'lider': {'gurultu_orani': 0.02, 'kayip_orani': 0.05, 'gecikme': 0.4},
                    'takipci': {'gurultu_orani': 0.12, 'kayip_orani': 0.15, 'gecikme': 0.5}
                },
                baslangic_hedefleri={
                    0: (40, 0, 60),
                    1: (35, -10, 50),
                    2: (40, -10, 50),
                    3: (45, -10, 50)
                },
                sensor_ayarlari={
                    'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
                    'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
                }
            )
        """
        # Varsayılan modem ayarları
        if modem_ayarlari is None:
            modem_ayarlari = {
                'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.1, 'gecikme': 0.5},
                'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1, 'gecikme': 0.5}
            }
        
        # Varsayılan sensör ayarları (sensor_ayarlari None ise otomatik uygulanır)
        if sensor_ayarlari is None:
            sensor_ayarlari = {
                'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0, 'kacinma_mesafesi': 4.0},
                'takipci': {'engel_mesafesi': 10.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0, 'kacinma_mesafesi': 4.0}
            }
        
        # Sensör ayarları için kontrol listesi
        varsayilan_sensor_ayarlari = {
            'engel_mesafesi': 10.0,
            'iletisim_menzili': 35.0,
            'min_pil_uyarisi': 10.0,
            'kacinma_mesafesi': 5.0
        }
        
        tum_modemler = {}
        lider_modem = None
        
        # Her ROV için işlem yap
        for i, rov in enumerate(rovs):
            # Sensör ayarlarını uygula
            if sensor_ayarlari:
                # Format 1: Tüm ROV'lar için ortak ayarlar (anahtar direkt sensör adı)
                if 'engel_mesafesi' in sensor_ayarlari or 'iletisim_menzili' in sensor_ayarlari or 'min_pil_uyarisi' in sensor_ayarlari:
                    for key, value in sensor_ayarlari.items():
                        if key in varsayilan_sensor_ayarlari:
                            rov.set(key, value)
                # Format 2: Lider ve takipçi için ayrı ayarlar
                elif 'lider' in sensor_ayarlari or 'takipci' in sensor_ayarlari:
                    if i == lider_id and 'lider' in sensor_ayarlari:
                        for key, value in sensor_ayarlari['lider'].items():
                            if key in varsayilan_sensor_ayarlari:
                                rov.set(key, value)
                    elif i != lider_id and 'takipci' in sensor_ayarlari:
                        for key, value in sensor_ayarlari['takipci'].items():
                            if key in varsayilan_sensor_ayarlari:
                                rov.set(key, value)
                # Format 3: Her ROV için özel ayarlar (ROV ID ile)
                elif isinstance(sensor_ayarlari, dict) and i in sensor_ayarlari:
                    for key, value in sensor_ayarlari[i].items():
                        if key in varsayilan_sensor_ayarlari:
                            rov.set(key, value)
            
            if i == lider_id:
                # Lider ROV
                rov.set("rol", 1)
                lider_modem = AkustikModem(
                    rov_id=i,
                    gurultu_orani=modem_ayarlari['lider'].get('gurultu_orani', 0.05),
                    kayip_orani=modem_ayarlari['lider'].get('kayip_orani', 0.1),
                    gecikme=modem_ayarlari['lider'].get('gecikme', 0.5)
                )
                rov.modem = lider_modem
                tum_modemler[i] = lider_modem
                
                # LiderGNC oluştur ve ekle (Filo referansı ile)
                gnc = LiderGNC(rov, lider_modem, filo_ref=self)
                self.ekle(gnc)
                
                # Başlangıç hedefi varsa ata (hedef_atama ile)
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    # (x, y, z) formatında
                    if len(hedef) >= 3:
                        gnc.hedef_atama(hedef[0], hedef[1], hedef[2])
                    else:
                        # Geriye uyumluluk: (x, z, y) formatı
                        gnc.hedef_atama(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
                elif baslangic_hedefleri is None:
                    # Varsayılan lider hedefi
                    gnc.hedef_atama(40, 0, 60)
            else:
                # Takipçi ROV
                rov.set("rol", 0)
                modem = AkustikModem(
                    rov_id=i,
                    gurultu_orani=modem_ayarlari['takipci'].get('gurultu_orani', 0.1),
                    kayip_orani=modem_ayarlari['takipci'].get('kayip_orani', 0.1),
                    gecikme=modem_ayarlari['takipci'].get('gecikme', 0.5)
                )
                rov.modem = modem
                tum_modemler[i] = modem
                
                # TakipciGNC oluştur ve ekle (lider_modem referansı ile)
                gnc = TakipciGNC(rov, modem, lider_modem_ref=lider_modem)
                self.ekle(gnc)
                
                # Başlangıç hedefi varsa ata (hedef_atama ile)
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    # (x, y, z) formatında
                    if len(hedef) >= 3:
                        gnc.hedef_atama(hedef[0], hedef[1], hedef[2])
                    else:
                        # Geriye uyumluluk: (x, z, y) formatı
                        gnc.hedef_atama(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
                else:
                    # Takipçi için hedef yoksa, liderin hedefine göre otomatik belirle
                    # Lider hedefi bul (lider zaten yukarıda oluşturuldu)
                    lider_gnc = self.sistemler[lider_id] if lider_id < len(self.sistemler) else None
                    if lider_gnc and lider_gnc.hedef:
                        # Lider hedefine göre formasyon
                        self._takipci_hedefi_belirle(gnc, i, lider_gnc.hedef.x, lider_gnc.hedef.y, lider_gnc.hedef.z, lider_id)
                    else:
                        # Lider hedefi henüz yoksa, varsayılan takipçi hedefi (formasyon)
                        offset_x = 30 + (i * 5)
                        gnc.hedef_atama(offset_x, -10, 50)
        
        # Rehberi dağıt
        self.rehber_dagit(tum_modemler)
        
        # Asıl hedefi belirle (orijinal liderin hedefi)
        if lider_id < len(self.sistemler):
            orijinal_lider_gnc = self.sistemler[lider_id]
            if orijinal_lider_gnc.hedef:
                self.asil_hedef = orijinal_lider_gnc.hedef
            elif baslangic_hedefleri and lider_id in baslangic_hedefleri:
                hedef = baslangic_hedefleri[lider_id]
                if len(hedef) >= 3:
                    self.asil_hedef = Vec3(hedef[0], hedef[1], hedef[2])
                else:
                    self.asil_hedef = Vec3(hedef[0], hedef[2] if len(hedef) > 2 else 0, hedef[1] if len(hedef) > 1 else 0)
            else:
                # Varsayılan hedef
                self.asil_hedef = Vec3(40, 0, 60)
        
        self.orijinal_lider_id = lider_id
        
        print(f"✅ GNC Sistemi Kuruldu: {len(rovs)} ROV (Lider: ROV-{lider_id})")
        
        return tum_modemler

    def guncelle_hepsi(self, tahminler):
        # Önce tüm GNC sistemlerini güncelle
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])
        
        # ÇOKLU LİDER DURUMU: Her grupta sadece bir lider olmalı (önce düzenle)
        self._coklu_lider_duzenle()
        
        # SÜRÜ AYRILMA TESPİTİ: Lideri olmayan sürüler için otomatik lider seçimi (sonra kontrol et)
        self._suru_ayrilma_tespiti()
    
    def _coklu_lider_duzenle(self):
        """
        Birden fazla lider durumunda: Her grupta sadece bir lider olmalı.
        Aynı grupta birden fazla lider varsa, asıl hedefe en yakın olan asıl liderdir.
        Farklı gruplardaki liderler birbirini etkilemez (iletişim kopmuş gruplar kendi liderlerini tutar).
        Ayrıca GNC tipini dinamik olarak günceller (TakipciGNC -> LiderGNC veya tersi).
        """
        # GNC tipini role göre güncelle
        for i, gnc in enumerate(self.sistemler):
            # TakipciGNC ama ROV lider olduysa -> LiderGNC'ye dönüştür
            if isinstance(gnc, TakipciGNC) and gnc.rov.role == 1:
                self._gnc_tipini_degistir(i, 'lider')
            # LiderGNC ama ROV takipçi olduysa -> TakipciGNC'ye dönüştür
            elif isinstance(gnc, LiderGNC) and gnc.rov.role == 0:
                # En yakın lideri bul
                en_yakin_lider_modem = self._en_yakin_lider_modem_bul(gnc.rov)
                self._gnc_tipini_degistir(i, 'takipci', lider_modem_ref=en_yakin_lider_modem)
        
        # İletişim bağlantılarına göre grupları bul
        gruplar = self._rov_gruplarini_bul()
        
        # Her grup için ayrı ayrı kontrol et (farklı gruplar birbirini etkilemez)
        for grup in gruplar:
            grup_liderleri = []
            for gnc_index in grup:
                gnc = self.sistemler[gnc_index]
                if gnc.rov.role == 1:
                    grup_liderleri.append((gnc_index, gnc))
            
            # Bir grupta birden fazla lider varsa, asıl hedefe en yakın olanı asıl lider yap
            # ÖNEMLİ: Sadece aynı gruptaki liderler arasında seçim yapılır
            if len(grup_liderleri) > 1:
                # Asıl hedef yoksa, orijinal liderin hedefini kullan
                if self.asil_hedef is None:
                    if self.orijinal_lider_id < len(self.sistemler):
                        orijinal_lider_gnc = self.sistemler[self.orijinal_lider_id]
                        if orijinal_lider_gnc.hedef:
                            self.asil_hedef = orijinal_lider_gnc.hedef
                        else:
                            self.asil_hedef = Vec3(40, 0, 60)
                
                # Asıl hedefe en yakın lideri bul (sadece bu grup içinde)
                asil_lider_index = None
                asil_lider_gnc = None
                en_yakin_mesafe = float('inf')
                
                for lider_index, lider_gnc in grup_liderleri:
                    mesafe = distance(lider_gnc.rov.position, self.asil_hedef)
                    if mesafe < en_yakin_mesafe:
                        en_yakin_mesafe = mesafe
                        asil_lider_index = lider_index
                        asil_lider_gnc = lider_gnc
                
                # Diğer liderleri takipçi yap (sadece bu grup içinde)
                # ÖNEMLİ: Sadece aynı gruptaki liderler etkilenir, farklı gruplardaki liderler korunur
                for lider_index, lider_gnc in grup_liderleri:
                    if lider_index != asil_lider_index:
                        # Sonsuz döngü önleme: Aynı ROV çok sık lider/takipçi değiştiriyorsa durdur
                        rov_id = lider_gnc.rov.id
                        if rov_id not in self.lider_degisim_sayaci:
                            self.lider_degisim_sayaci[rov_id] = 0
                        
                        self.lider_degisim_sayaci[rov_id] += 1
                        
                        # Eğer son 10 frame'de 5 kereden fazla değiştiyse, durdur (sonsuz döngü)
                        if self.lider_degisim_sayaci[rov_id] > 5:
                            # Bu ROV'u korunan listesine ekle (sadece bir kez)
                            if rov_id not in self.korunan_rovlar:
                                print(f"⚠️ [SONSUZ DÖNGÜ ÖNLEME] ROV-{rov_id} çok sık lider/takipçi değiştiriyor, lider olarak bırakılıyor")
                                self.korunan_rovlar.add(rov_id)
                            # Sayacı sıfırla ve bu ROV'u lider olarak bırak (sonsuz döngüyü durdur)
                            self.lider_degisim_sayaci[rov_id] = 0
                            # Bu ROV'u lider olarak bırak, takipçi yapma
                            continue
                        
                        # Eğer bu ROV korunan listede ise, takipçi yapma
                        if rov_id in self.korunan_rovlar:
                            continue
                        
                        # ÖNEMLİ: Eğer bu ROV asıl liderle iletişim kuruyorsa, takipçi yapma
                        # Bu, geçici iletişim kopmalarını önler
                        asil_lider_rov = asil_lider_gnc.rov
                        mesafe = distance(lider_gnc.rov.position, asil_lider_rov.position)
                        rov_yuzeyde = lider_gnc.rov.y >= 0
                        asil_lider_yuzeyde = asil_lider_rov.y >= 0
                        
                        iletisim_var = False
                        if rov_yuzeyde and asil_lider_yuzeyde:
                            iletisim_var = True
                        else:
                            iletisim_menzili = lider_gnc.rov.sensor_config.get("iletisim_menzili", 35.0)
                            # Çok yakınsa (10m içinde) iletişim var say
                            if mesafe < 10.0:
                                iletisim_var = True
                            else:
                                iletisim_var = mesafe < iletisim_menzili
                        
                        # Eğer iletişim varsa, takipçi yapma (geçici kopmaları önle)
                        if iletisim_var:
                            continue
                        
                        # Sadece aynı gruptaki liderleri takipçi yap
                        # Farklı gruplardaki liderler (iletişim kopmuş) korunur
                        print(f"🔄 [SÜRÜ LİDER] ROV-{lider_gnc.rov.id} aynı grupta asıl hedefe uzak, takipçi yapılıyor (Asıl lider: ROV-{asil_lider_gnc.rov.id})")
                        lider_gnc.rov.set("rol", 0)
                        # Asıl liderin modem referansını al
                        asil_lider_modem = asil_lider_gnc.modem if asil_lider_gnc.modem else None
                        self._gnc_tipini_degistir(lider_index, 'takipci', lider_modem_ref=asil_lider_modem)
                        
                        # Takipçi hedefini asıl liderin yakınına ayarla
                        takipci_gnc = self.sistemler[lider_index]
                        if isinstance(takipci_gnc, TakipciGNC):
                            offset_x = (lider_index - asil_lider_index) * 5.0  # Formasyon offset
                            takipci_gnc.hedef = Vec3(
                                asil_lider_gnc.rov.x + offset_x,
                                asil_lider_gnc.rov.y,
                                asil_lider_gnc.rov.z
                            )
                        
                        # Lider değişim sayacını artır (takipçi yapıldığı için)
                        self.lider_degisim_sayaci[rov_id] += 0.5
        
        # Lider değişim sayacını azalt (her frame'de bir)
        # Bu sayede sürekli değişen ROV'lar tespit edilir
        for rov_id in list(self.lider_degisim_sayaci.keys()):
            if self.lider_degisim_sayaci[rov_id] > 0:
                self.lider_degisim_sayaci[rov_id] = max(0, self.lider_degisim_sayaci[rov_id] - 0.1)
                # Eğer sayac 0'a düştüyse, dict'ten kaldır
                if self.lider_degisim_sayaci[rov_id] <= 0:
                    del self.lider_degisim_sayaci[rov_id]
        
        # Tüm liderleri bul (takipçi bağlantıları için)
        liderler = []
        for i, gnc in enumerate(self.sistemler):
            if gnc.rov.role == 1:
                liderler.append((i, gnc))
        
        # Her takipçi için en yakın iletişim menzilindeki lideri bul
        if len(liderler) > 0:
            # Her takipçi için en yakın iletişim menzilindeki lideri bul
            for i, gnc in enumerate(self.sistemler):
                if gnc.rov.role == 0 and isinstance(gnc, TakipciGNC):  # Takipçi ise
                    en_yakin_lider = None
                    en_yakin_mesafe = float('inf')
                    
                    for lider_id, lider_gnc in liderler:
                        lider_rov = lider_gnc.rov
                        mesafe = distance(gnc.rov.position, lider_rov.position)
                        
                        # Yüzey iletişimi kontrolü
                        gnc_yuzeyde = gnc.rov.y >= 0
                        lider_yuzeyde = lider_rov.y >= 0
                        
                        iletisim_var = False
                        if gnc_yuzeyde and lider_yuzeyde:
                            # Yüzey iletişimi sınırsız
                            iletisim_var = True
                        else:
                            # Su altı iletişimi
                            iletisim_menzili = gnc.rov.sensor_config.get("iletisim_menzili", 35.0)
                            iletisim_var = mesafe < iletisim_menzili
                        
                        # İletişim menzilindeyse ve daha yakınsa
                        if iletisim_var and mesafe < en_yakin_mesafe:
                            en_yakin_mesafe = mesafe
                            en_yakin_lider = lider_gnc
                    
                    # En yakın liderle bağlantıyı güncelle (TakipciGNC için lider_ref)
                    if en_yakin_lider:
                        gnc.lider_ref = en_yakin_lider.modem if en_yakin_lider.modem else None
    
    def _takipci_hedefi_belirle(self, takipci_gnc, takipci_rov_id, lider_x, lider_y, lider_z, lider_rov_id):
        """
        Tek bir takipçi ROV için hedef belirler (liderin hedefine göre +-10 metre mesafede).
        
        Args:
            takipci_gnc: Takipçi GNC objesi
            takipci_rov_id: Takipçi ROV'un ID'si
            lider_x: Lider hedef X koordinatı
            lider_y: Lider hedef Y koordinatı (derinlik)
            lider_z: Lider hedef Z koordinatı
            lider_rov_id: Lider ROV'un ID'si
        """
        formasyon_mesafesi = 10.0  # +-10 metre
        
        # Formasyon offset'leri (her takipçi için farklı pozisyon)
        # Basit formasyon: Lider merkezde, takipçiler çevresinde
        formasyon_offsetleri = [
            (-formasyon_mesafesi, -formasyon_mesafesi),  # Takipçi 1: Sol-Alt
            (formasyon_mesafesi, -formasyon_mesafesi),   # Takipçi 2: Sağ-Alt
            (-formasyon_mesafesi, formasyon_mesafesi),   # Takipçi 3: Sol-Üst
            (formasyon_mesafesi, formasyon_mesafesi),   # Takipçi 4: Sağ-Üst
            (0, -formasyon_mesafesi),                    # Takipçi 5: Alt
            (0, formasyon_mesafesi),                     # Takipçi 6: Üst
            (-formasyon_mesafesi, 0),                    # Takipçi 7: Sol
            (formasyon_mesafesi, 0),                     # Takipçi 8: Sağ
        ]
        
        # Takipçi index'i: Lider hariç, takipçilerin sırası
        takipci_index = 0
        for i, gnc in enumerate(self.sistemler):
            if i == lider_rov_id:
                continue
            if i == takipci_rov_id:
                break
            if gnc.rov.role == 0:  # Takipçi ise
                takipci_index += 1
        
        # Formasyon offset'ini al (eğer takipçi sayısı offset sayısından fazlaysa, tekrar kullan)
        offset_x, offset_z = formasyon_offsetleri[takipci_index % len(formasyon_offsetleri)]
        
        # Takipçi hedefi: Lider hedefi + offset
        takipci_x = lider_x + offset_x
        takipci_z = lider_z + offset_z
        takipci_y = lider_y  # Aynı derinlik (veya -10 gibi sabit bir değer)
        
        # Eğer lider yüzeydeyse (y >= 0), takipçiler su altında olmalı
        if lider_y >= 0:
            takipci_y = -10.0  # Su altı derinliği
        
        # Hedef atama
        try:
            takipci_gnc.hedef_atama(takipci_x, takipci_y, takipci_z)
        except Exception as e:
            print(f"⚠️ [UYARI] ROV-{takipci_rov_id} hedefi belirlenirken hata: {e}")
    
    def _takipci_hedeflerini_guncelle(self, lider_rov_id, lider_x, lider_y, lider_z):
        """
        Lider ROV'un hedefi değiştiğinde, tüm takipçi ROV'ların hedeflerini
        liderin hedefine göre +-10 metre mesafede formasyon şeklinde günceller.
        
        Args:
            lider_rov_id: Lider ROV'un ID'si
            lider_x: Lider hedef X koordinatı
            lider_y: Lider hedef Y koordinatı (derinlik)
            lider_z: Lider hedef Z koordinatı
        """
        for i, gnc in enumerate(self.sistemler):
            # Lider ROV'u atla
            if i == lider_rov_id:
                continue
            
            # Sadece takipçi ROV'lar için hedef güncelle
            if gnc.rov.role == 0:  # Takipçi ise
                self._takipci_hedefi_belirle(gnc, i, lider_x, lider_y, lider_z, lider_rov_id)
                print(f"✅ [FİLO] ROV-{i} hedefi otomatik güncellendi: Lider hedefine göre formasyon")
    
    def _gnc_tipini_degistir(self, index, yeni_tip, lider_modem_ref=None):
        """
        GNC tipini dinamik olarak değiştirir.
        
        Args:
            index: GNC sisteminin indeksi
            yeni_tip: 'lider' veya 'takipci'
            lider_modem_ref: TakipciGNC için lider modem referansı
        """
        if index >= len(self.sistemler):
            return
        
        mevcut_gnc = self.sistemler[index]
        rov = mevcut_gnc.rov
        modem = mevcut_gnc.modem
        
        # Aynı tipteyse değiştirme
        if (yeni_tip == 'lider' and isinstance(mevcut_gnc, LiderGNC)) or \
           (yeni_tip == 'takipci' and isinstance(mevcut_gnc, TakipciGNC)):
            return
        
        # Yeni GNC oluştur
        if yeni_tip == 'lider':
            # Filo referansını bul (self Filo sınıfının instance'ı)
            filo_ref = self if isinstance(self, Filo) else None
            # Eğer mevcut GNC'de Filo referansı varsa onu kullan
            if isinstance(mevcut_gnc, LiderGNC) and hasattr(mevcut_gnc, 'filo_ref') and mevcut_gnc.filo_ref:
                filo_ref = mevcut_gnc.filo_ref
            yeni_gnc = LiderGNC(rov, modem, filo_ref=filo_ref)
        else:  # takipci
            yeni_gnc = TakipciGNC(rov, modem, lider_modem_ref=lider_modem_ref)
        
        # Özellikleri kopyala
        yeni_gnc.hedef = mevcut_gnc.hedef
        yeni_gnc.hiz_limiti = mevcut_gnc.hiz_limiti
        yeni_gnc.manuel_kontrol = mevcut_gnc.manuel_kontrol
        yeni_gnc.ai_aktif = mevcut_gnc.ai_aktif
        
        # LiderGNC özel özellikleri
        if isinstance(yeni_gnc, LiderGNC) and isinstance(mevcut_gnc, LiderGNC):
            yeni_gnc.diger_lider_hedefi = mevcut_gnc.diger_lider_hedefi
            yeni_gnc.diger_lider_bulundu = mevcut_gnc.diger_lider_bulundu
        
        # Eski GNC'yi yeni ile değiştir
        self.sistemler[index] = yeni_gnc
    
    def _en_yakin_lider_modem_bul(self, rov):
        """En yakın lider ROV'un modem referansını bulur"""
        en_yakin_lider_modem = None
        en_yakin_mesafe = float('inf')
        
        for gnc in self.sistemler:
            if gnc.rov.role == 1 and gnc.rov.id != rov.id:  # Lider ve kendisi değil
                mesafe = distance(rov.position, gnc.rov.position)
                if mesafe < en_yakin_mesafe:
                    en_yakin_mesafe = mesafe
                    en_yakin_lider_modem = gnc.modem
        
        return en_yakin_lider_modem
    
    def _suru_ayrilma_tespiti(self):
        """
        Sürü ayrılma tespiti: Lideri olmayan sürüler için otomatik lider seçimi.
        Bir sürüde sadece bir lider olabilir. Asıl hedefe en yakın olan lider seçilir.
        Seçilen lider yüzeye çıkar ve diğer liderle iletişim kurup ona doğru ilerler.
        """
        if len(self.sistemler) == 0:
            return
        
        # İletişim bağlantılarına göre ROV gruplarını bul
        gruplar = self._rov_gruplarini_bul()
        
        # Her grup için kontrol et
        for grup_id, grup in enumerate(gruplar):
            # Grubun lideri var mı?
            grup_lideri = None
            for gnc_index in grup:
                gnc = self.sistemler[gnc_index]
                if gnc.rov.role == 1:
                    grup_lideri = gnc_index
                    # Bu grup için lider var, flag'i temizle
                    if grup_id in self.suru_lider_secildi:
                        del self.suru_lider_secildi[grup_id]
                    break
            
            # Lideri yoksa ve daha önce lider seçilmemişse, asıl hedefe en yakın olanı lider seç
            # ÖNEMLİ: Sadece gerçekten lideri olmayan gruplar için lider seç
            if grup_lideri is None and len(grup) > 1 and grup_id not in self.suru_lider_secildi:
                # Asıl hedef yoksa, orijinal liderin hedefini kullan
                if self.asil_hedef is None:
                    if self.orijinal_lider_id < len(self.sistemler):
                        orijinal_lider_gnc = self.sistemler[self.orijinal_lider_id]
                        if orijinal_lider_gnc.hedef:
                            self.asil_hedef = orijinal_lider_gnc.hedef
                        else:
                            self.asil_hedef = Vec3(40, 0, 60)
                
                # Asıl hedefe en yakın ROV'u lider seç
                yeni_lider_index = None
                en_yakin_mesafe = float('inf')
                
                for gnc_index in grup:
                    gnc = self.sistemler[gnc_index]
                    mesafe = distance(gnc.rov.position, self.asil_hedef)
                    if mesafe < en_yakin_mesafe:
                        en_yakin_mesafe = mesafe
                        yeni_lider_index = gnc_index
                
                if yeni_lider_index is None:
                    continue
                
                yeni_lider_gnc = self.sistemler[yeni_lider_index]
                
                # Sonsuz döngü önleme: Eğer bu ROV son zamanlarda çok sık lider/takipçi değiştirdiyse, lider seçme
                rov_id = yeni_lider_gnc.rov.id
                if rov_id in self.lider_degisim_sayaci and self.lider_degisim_sayaci[rov_id] > 3:
                    # Bu ROV çok sık değişiyor, lider seçme
                    continue
                
                print(f"🔴 [SÜRÜ AYRILMA] Grup lideri yok! ROV-{yeni_lider_gnc.rov.id} asıl hedefe en yakın, lider seçildi.")
                
                # Bu grup için lider seçildi flag'ini ayarla
                self.suru_lider_secildi[grup_id] = True
                
                # ROV'u lider yap
                yeni_lider_gnc.rov.set("rol", 1)
                
                # GNC tipini değiştir
                self._gnc_tipini_degistir(yeni_lider_index, 'lider')
                
                # Lideri yüzeye çıkar (hedef yüzeye ayarla)
                yeni_lider_gnc.hedef = Vec3(
                    yeni_lider_gnc.rov.x,
                    0.0,  # Yüzey
                    yeni_lider_gnc.rov.z
                )
                
                # Diğer grup üyelerini takipçi yap ve yeni lideri takip etmelerini sağla
                for gnc_index in grup:
                    if gnc_index != yeni_lider_index:
                        gnc = self.sistemler[gnc_index]
                        if gnc.rov.role == 1:  # Eğer lider ise takipçi yap
                            gnc.rov.set("rol", 0)
                        
                        # Yeni liderin modem referansını al
                        yeni_lider_modem = yeni_lider_gnc.modem if yeni_lider_gnc.modem else None
                        
                        # GNC tipini takipçi yap
                        self._gnc_tipini_degistir(gnc_index, 'takipci', lider_modem_ref=yeni_lider_modem)
                        
                        # Takipçi hedefini yeni liderin yakınına ayarla (formasyon)
                        takipci_gnc = self.sistemler[gnc_index]
                        if isinstance(takipci_gnc, TakipciGNC):
                            # Yeni liderin konumuna göre formasyon hedefi
                            offset_x = (gnc_index - yeni_lider_index) * 5.0  # Formasyon offset
                            takipci_gnc.hedef = Vec3(
                                yeni_lider_gnc.rov.x + offset_x,
                                yeni_lider_gnc.rov.y,
                                yeni_lider_gnc.rov.z
                            )
    
    def _rov_gruplarini_bul(self):
        """
        İletişim bağlantılarına göre ROV'ları gruplara ayırır.
        İletişim kurabilen ROV'lar aynı gruptadır.
        
        Returns:
            list: Her grup bir liste içinde GNC indekslerini içerir
        """
        n = len(self.sistemler)
        if n == 0:
            return []
        
        # İletişim grafiği oluştur
        iletisim_grafi = {}
        for i in range(n):
            iletisim_grafi[i] = []
        
        # Her ROV çifti için iletişim kontrolü
        for i in range(n):
            for j in range(i + 1, n):
                gnc_i = self.sistemler[i]
                gnc_j = self.sistemler[j]
                rov_i = gnc_i.rov
                rov_j = gnc_j.rov
                
                mesafe = distance(rov_i.position, rov_j.position)
                rov_i_yuzeyde = rov_i.y >= 0
                rov_j_yuzeyde = rov_j.y >= 0
                
                iletisim_var = False
                # Yüzey iletişimi
                if rov_i_yuzeyde and rov_j_yuzeyde:
                    iletisim_var = True
                # Su altı iletişimi
                else:
                    iletisim_menzili = rov_i.sensor_config.get("iletisim_menzili", 35.0)
                    iletisim_var = mesafe < iletisim_menzili
                
                if iletisim_var:
                    iletisim_grafi[i].append(j)
                    iletisim_grafi[j].append(i)
        
        # BFS ile grupları bul
        ziyaret_edildi = [False] * n
        gruplar = []
        
        for i in range(n):
            if not ziyaret_edildi[i]:
                # Yeni grup başlat
                grup = []
                kuyruk = [i]
                ziyaret_edildi[i] = True
                
                while kuyruk:
                    mevcut = kuyruk.pop(0)
                    grup.append(mevcut)
                    
                    # Komşuları ekle
                    for komsu in iletisim_grafi[mevcut]:
                        if not ziyaret_edildi[komsu]:
                            ziyaret_edildi[komsu] = True
                            kuyruk.append(komsu)
                
                gruplar.append(grup)
        
        return gruplar
    
    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            ayar_adi: Ayar adı ('rol', 'renk', 'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'kacinma_mesafesi')
            deger: Ayar değeri
        
        Örnekler:
            filo.set(0, 'rol', 1)  # ROV-0'ı lider yap
            filo.set(1, 'renk', (255, 0, 0))  # ROV-1'i kırmızı yap
            filo.set(2, 'engel_mesafesi', 30.0)  # ROV-2'nin engel mesafesini ayarla
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return False
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return False
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return False
        
        try:
            rov = self.sistemler[rov_id].rov
            rov.set(ayar_adi, deger)
            print(f"✅ [FİLO] ROV-{rov_id} ayarı güncellendi: {ayar_adi} = {deger}")
            return True
        except Exception as e:
            print(f"❌ [HATA] Ayar güncelleme sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return False

    def get(self, rov_id, veri_tipi):
        """
        ROV bilgilerini alır.
        
        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            veri_tipi: Veri tipi ('gps', 'hiz', 'batarya', 'rol', 'renk', 'sensör', 
                                  'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'kacinma_mesafesi', 'sonar')
        
        Returns:
            İstenen veri tipine göre değer
        
        Örnekler:
            pozisyon = filo.get(0, 'gps')
            rol = filo.get(1, 'rol')
            sensörler = filo.get(2, 'sensör')
            batarya = filo.get(0, 'batarya')
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            return None
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return None
        
        try:
            rov = self.sistemler[rov_id].rov
            deger = rov.get(veri_tipi)
            if deger is None:
                print(f"⚠️ [UYARI] ROV-{rov_id} için '{veri_tipi}' veri tipi bulunamadı")
            return deger
        except Exception as e:
            print(f"❌ [HATA] Veri alma sırasında hata: {e}")
            import traceback
            traceback.print_exc()
            return None

    def git(self, rov_id, x, z, y=None, ai=True):
        """
        ROV'a hedef koordinatı atar ve otomatik moda geçirir.

        Args:
            rov_id: ROV ID (0, 1, 2, ...)
            x: X koordinatı (yatay düzlem)
            z: Z koordinatı (yatay düzlem)
            y: Y koordinatı (derinlik, negatif = su altı, opsiyonel)
                - None ise mevcut derinlik korunur
            ai: AI aktif/pasif (varsayılan: True)
                - True: Zeki Mod (GAT tahminleri kullanılır)
                - False: Kör Mod (GAT tahminleri görmezden gelinir)

        Örnekler:
            filo.git(0, 40, 60, 0)           # ROV-0: (40, 0, 60), AI açık
            filo.git(1, 50, 50, -10, ai=False)  # ROV-1: (50, -10, 50), AI kapalı
            filo.git(2, 30, 40)               # ROV-2: (30, mevcut_y, 40), AI açık
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            print(f"   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return
        
        # Manuel modu kapat, otopilotu aç
        self.sistemler[rov_id].manuel_kontrol = False
        
        # AI Durumunu Ayarla
        self.sistemler[rov_id].ai_aktif = ai
        
        # Hedef Ata
        # Eğer y belirtilmemişse, mevcut derinliği koru
        if y is None:
            hedef_y = self.sistemler[rov_id].rov.y
        else:
            hedef_y = y
        
        # Bilgilendirme mesajı
        ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
        print(f"🔵 [FİLO] ROV-{rov_id} Rota: ({x}, {hedef_y}, {z}) | AI: {ai_durum}")
        
        # Hedef atama (x, y, z formatında)
        try:
            self.sistemler[rov_id].hedef_atama(x, hedef_y, z)
            print(f"✅ [FİLO] ROV-{rov_id} hedefi başarıyla atandı")
            
            # Eğer lider ROV'a hedef verildiyse, takipçilerin hedeflerini otomatik güncelle
            if self.sistemler[rov_id].rov.role == 1:  # Lider ise
                self._takipci_hedeflerini_guncelle(rov_id, x, hedef_y, z)
        except Exception as e:
            print(f"❌ [HATA] Hedef atama sırasında hata: {e}")
            import traceback
            traceback.print_exc()

    def move(self, rov_id, yon, guc=1.0):
        """
        ROV'a güç bazlı hareket komutu verir (gerçek dünya gibi, gerçekçi fizik ile).
        
        Args:
            rov_id: ROV ID
            yon: Hareket yönü ('ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur')
            guc: Motor gücü (0.0 - 1.0 arası, varsayılan: 1.0)
                - 1.0 = %100 güç (maksimum hız)
                - 0.5 = %50 güç (yarı hız)
                - 0.0 = %0 güç (dur)
        
        Örnekler:
            filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
            filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
            filo.move(2, 'cik', 0.3)      # ROV-2 %30 güçle yukarı
            filo.move(3, 'dur', 0.0)      # ROV-3 dur (güç=0)
            filo.move(0, 'ileri')         # ROV-0 %100 güçle ileri (varsayılan)
        """
        # Sistemler listesi boş mu kontrol et
        if len(self.sistemler) == 0:
            print(f"❌ [HATA] GNC sistemleri henüz kurulmamış!")
            print(f"   💡 Çözüm: filo.ekle() ile GNC sistemleri ekleyin")
            return
        
        # ROV ID geçerliliği kontrolü
        if not isinstance(rov_id, int) or rov_id < 0:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id} (pozitif tam sayı olmalı)")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            return
        
        if rov_id >= len(self.sistemler):
            print(f"❌ [HATA] ROV ID {rov_id} mevcut değil!")
            print(f"   Mevcut ROV sayısı: {len(self.sistemler)} (0-{len(self.sistemler)-1} arası)")
            print(f"   💡 Çözüm: filo.ekle() ile daha fazla GNC sistemi ekleyin")
            return
        
        # Yön geçerliliği kontrolü
        gecerli_yonler = ['ileri', 'geri', 'sag', 'sol', 'cik', 'bat', 'dur']
        if yon not in gecerli_yonler:
            print(f"❌ [HATA] Geçersiz hareket yönü: '{yon}'")
            print(f"   Geçerli yönler: {', '.join(gecerli_yonler)}")
            return
        
        # Güç değerini kontrol et (0.0 - 1.0 arası)
        if not isinstance(guc, (int, float)):
            print(f"❌ [HATA] Güç değeri sayı olmalı: {guc}")
            return
        
        guc = max(0.0, min(1.0, float(guc)))
        
        try:
            # Manuel kontrolü aç
            self.sistemler[rov_id].manuel_kontrol = True
            gnc = self.sistemler[rov_id]
            rov = gnc.rov
            
            # 'dur' komutu özel durum
            if yon == 'dur' or guc == 0.0:
                if hasattr(rov, 'manuel_hareket'):
                    rov.manuel_hareket['yon'] = None
                    rov.manuel_hareket['guc'] = 0.0
                rov.velocity *= 0.9  # Yavaşça dur (momentum korunumu)
                print(f"🛑 [FİLO] ROV-{rov_id} durduruluyor")
                return
            
            # Lider ROV batırılamaz kontrolü
            if yon == 'bat' and rov.role == 1:
                print(f"⚠️ [FİLO] ROV-{rov_id} lider, batırılamaz!")
                return
            
            # Havuz sınır kontrolü (hareket öncesi)
            if hasattr(rov, 'environment_ref') and rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_yari_genislik = havuz_genisligi / 2
                
                # Sınırda mı kontrol et
                sinirda_x = abs(rov.x) >= havuz_yari_genislik * 0.95
                sinirda_z = abs(rov.z) >= havuz_yari_genislik * 0.95
                sinirda_y_ust = rov.y >= 0.3
                sinirda_y_alt = rov.y <= -95
                
                # Sınırda ise o yöne hareketi engelle
                if sinirda_x and ((yon == 'sag' and rov.x > 0) or (yon == 'sol' and rov.x < 0)):
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (X), {yon} yönünde hareket engellendi")
                    return
                
                if sinirda_z and ((yon == 'ileri' and rov.z > 0) or (yon == 'geri' and rov.z < 0)):
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırında (Z), {yon} yönünde hareket engellendi")
                    return
                
                if sinirda_y_ust and yon == 'cik':
                    print(f"⚠️ [FİLO] ROV-{rov_id} su yüzeyinde, yukarı hareket engellendi")
                    return
                
                if sinirda_y_alt and yon == 'bat':
                    print(f"⚠️ [FİLO] ROV-{rov_id} deniz tabanında, aşağı hareket engellendi")
                    return
            
            # Manuel hareket modunu ayarla (sürekli hareket için)
            if hasattr(rov, 'manuel_hareket'):
                rov.manuel_hareket['yon'] = yon
                rov.manuel_hareket['guc'] = guc
                guc_yuzdesi = int(guc * 100)
                print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor (sürekli mod)")
                return
            
            # Alternatif: ROV'un move metodunu kullan (manuel_hareket yoksa)
            if hasattr(rov, 'move'):
                try:
                    rov.move(yon, guc)
                    guc_yuzdesi = int(guc * 100)
                    print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor")
                    return
                except Exception as e:
                    # ROV.move() başarısız oldu, alternatif yöntem kullan
                    pass
            
            # Son alternatif: Direkt velocity kullan
            hareket_vektoru = Vec3(0, 0, 0)
            if yon == 'ileri': hareket_vektoru.z = 1.0
            elif yon == 'geri': hareket_vektoru.z = -1.0
            elif yon == 'sag': hareket_vektoru.x = 1.0
            elif yon == 'sol': hareket_vektoru.x = -1.0
            elif yon == 'cik': hareket_vektoru.y = 1.0
            elif yon == 'bat' and rov.role != 1: hareket_vektoru.y = -1.0
            
            # Hız uygula
            max_guc = 100.0 * guc
            if hareket_vektoru.length() > 0:
                rov.velocity += hareket_vektoru.normalized() * max_guc * time.dt * 0.5
                
                # Hız limiti
                if rov.velocity.length() > max_guc:
                    rov.velocity = rov.velocity.normalized() * max_guc
            
            guc_yuzdesi = int(guc * 100)
            print(f"🔵 [FİLO] ROV-{rov_id} {yon} yönünde %{guc_yuzdesi} güçle hareket ediyor")
            
        except AttributeError as e:
            print(f"❌ [HATA] ROV-{rov_id} için gerekli özellik bulunamadı: {e}")
            print(f"   💡 Debug: GNC sistemi tipi: {type(self.sistemler[rov_id])}")
            import traceback
            traceback.print_exc()
        except Exception as e:
            print(f"❌ [HATA] Hareket komutu sırasında hata: {e}")
            print(f"   💡 Debug: ROV ID: {rov_id}, Yön: {yon}, Güç: {guc}")
            import traceback
            traceback.print_exc()

# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    def __init__(self, rov_entity, modem):
        self.rov = rov_entity
        self.modem = modem
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        
        # YENİ: Bireysel AI Anahtarı
        self.ai_aktif = True 

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: self.modem.rehber_guncelle(rehber)

    def vektor_to_motor(self, vektor, guc_carpani=1.0):
        if vektor.length() == 0: return

        # Güç çarpanını normalize et (0.0-1.0 arası)
        guc_carpani = max(0.0, min(1.0, guc_carpani))
        
        # Vektörü normalize et (eğer normalize edilmemişse)
        vektor_magnitude = vektor.length()
        if vektor_magnitude > 0:
            vektor_normalized = vektor / vektor_magnitude
        else:
            return

        # Her bileşen için güç hesapla
        # Normalize edilmiş vektör bileşenleri zaten 0.0-1.0 arası
        # Ama diagonal hareketlerde bileşenler küçük olabilir (örn: 0.707)
        # Bu yüzden vektörün büyüklüğünü de dikkate alıyoruz
        if vektor_normalized.x > 0.1: 
            # Bileşen değerini direkt kullan (zaten normalize edilmiş)
            guc_x = abs(vektor_normalized.x) * guc_carpani
            self.rov.move("sag", guc_x)
        elif vektor_normalized.x < -0.1: 
            guc_x = abs(vektor_normalized.x) * guc_carpani
            self.rov.move("sol", guc_x)

        if vektor_normalized.y > 0.1: 
            guc_y = abs(vektor_normalized.y) * guc_carpani
            self.rov.move("cik", guc_y)
        elif vektor_normalized.y < -0.1: 
            guc_y = abs(vektor_normalized.y) * guc_carpani
            self.rov.move("bat", guc_y)

        if vektor_normalized.z > 0.1: 
            guc_z = abs(vektor_normalized.z) * guc_carpani
            self.rov.move("ileri", guc_z)
        elif vektor_normalized.z < -0.1: 
            guc_z = abs(vektor_normalized.z) * guc_carpani
            self.rov.move("geri", guc_z)
    
    def _yaklasma_onleme_vektoru(self, gat_kodu=0, hedef_vektoru=None):
        """
        Sensör mesafesine göre ROV'lar ve engellerden uzaklaşma vektörü.
        GAT kodlarına göre en uygun rotayı hesaplar.
        
        Args:
            gat_kodu: GAT kod (0=OK, 1=ENGEL, 2=CARPISMA, 3=KOPUK, 5=UZAK)
            hedef_vektoru: Hedef yönü vektörü (opsiyonel)
        
        Returns:
            Vec3: Kaçınma vektörü (en uygun rota)
        """
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return Vec3(0, 0, 0)
        
        # Kaçınma mesafesini sensör ayarlarından al (veya engel_mesafesi kullan)
        kacinma_mesafesi = self.rov.sensor_config.get("kacinma_mesafesi", None)
        if kacinma_mesafesi is None:
            # Eğer kacinma_mesafesi yoksa, engel_mesafesi'nin bir kısmını kullan
            engel_mesafesi = self.rov.sensor_config.get("engel_mesafesi", 20.0)
            kacinma_mesafesi = engel_mesafesi * 0.2  # Engel mesafesinin %20'si
        
        # Hedef vektörü hesapla
        if hedef_vektoru is None:
            if self.hedef:
                hedef_vektoru = (self.hedef - self.rov.position)
                if hedef_vektoru.length() > 0:
                    hedef_vektoru = hedef_vektoru.normalized()
                else:
                    hedef_vektoru = Vec3(0, 0, 0)
            else:
                hedef_vektoru = Vec3(0, 0, 0)
        
        # Tehlikeli nesneleri tespit et (ROV'lar ve engeller)
        tehlikeli_nesneler = []
        
        # Diğer ROV'lar
        for diger_rov in self.rov.environment_ref.rovs:
            if diger_rov.id == self.rov.id:
                continue
            mesafe = distance(self.rov.position, diger_rov.position)
            if mesafe <= kacinma_mesafesi and mesafe > 0:
                tehlikeli_nesneler.append({
                    'pozisyon': diger_rov.position,
                    'mesafe': mesafe,
                    'tip': 'rov'
                })
        
        # Engeller
        for engel in self.rov.environment_ref.engeller:
            mesafe = distance(self.rov.position, engel.position)
            engel_yari_cap = max(engel.scale_x, engel.scale_y, engel.scale_z) / 2
            gercek_mesafe = mesafe - engel_yari_cap
            if gercek_mesafe <= kacinma_mesafesi and gercek_mesafe > 0:
                tehlikeli_nesneler.append({
                    'pozisyon': engel.position,
                    'mesafe': gercek_mesafe,
                    'tip': 'engel'
                })
        
        # Eğer tehlikeli nesne yoksa, boş vektör döndür
        if len(tehlikeli_nesneler) == 0:
            return Vec3(0, 0, 0)
        
        # GAT KOD 2 (ÇARPISMA): En uygun rotayı bul (yukarı çıkmak yerine)
        if gat_kodu == 2:
            return self._en_uygun_rota_bul(tehlikeli_nesneler, hedef_vektoru, kacinma_mesafesi)
        
        # GAT KOD 1 (ENGEL): Engelden uzaklaş + hedefe doğru yönel
        if gat_kodu == 1:
            uzaklasma_vektoru = Vec3(0, 0, 0)
            for nesne in tehlikeli_nesneler:
                uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
                uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
            
            # Hedef yönüne de yönel
            if hedef_vektoru.length() > 0:
                uzaklasma_vektoru = uzaklasma_vektoru + hedef_vektoru * 0.3
            
            if uzaklasma_vektoru.length() > 0:
                return uzaklasma_vektoru.normalized()
            return Vec3(0, 0, 0)
        
        # Normal kaçınma: Tehlikeli nesnelerden uzaklaş
        uzaklasma_vektoru = Vec3(0, 0, 0)
        for nesne in tehlikeli_nesneler:
            uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
            uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
            uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
        
        # Hedef yönüne de yönel (kaçınma ile birleştir)
        if hedef_vektoru.length() > 0 and uzaklasma_vektoru.length() > 0:
            # Kaçınma vektörünü normalize et
            uzaklasma_vektoru = uzaklasma_vektoru.normalized()
            # Hedef yönünü ekle (daha az ağırlıkla)
            nihai_vektor = uzaklasma_vektoru * 0.7 + hedef_vektoru * 0.3
            return nihai_vektor.normalized()
        
        if uzaklasma_vektoru.length() > 0:
            return uzaklasma_vektoru.normalized()
        
        return Vec3(0, 0, 0)
    
    def _en_uygun_rota_bul(self, tehlikeli_nesneler, hedef_vektoru, kacinma_mesafesi):
        """
        GAT kod 2 (çarpışma) için en uygun rotayı bulur.
        Yukarı çıkmak yerine, engeller ve ROV'lar arasından en güvenli yolu seçer.
        """
        # Farklı yönleri test et (8 yön: ileri, geri, sağ, sol, çaprazlar)
        test_yonleri = [
            Vec3(1, 0, 0),   # Sağ
            Vec3(-1, 0, 0),  # Sol
            Vec3(0, 0, 1),   # İleri
            Vec3(0, 0, -1),  # Geri
            Vec3(1, 0, 1).normalized(),   # Sağ-İleri
            Vec3(-1, 0, 1).normalized(),  # Sol-İleri
            Vec3(1, 0, -1).normalized(),  # Sağ-Geri
            Vec3(-1, 0, -1).normalized(), # Sol-Geri
        ]
        
        # Hedef yönünü de ekle (eğer varsa)
        if hedef_vektoru.length() > 0:
            # Yatay düzlemde (y=0)
            hedef_yatay = Vec3(hedef_vektoru.x, 0, hedef_vektoru.z)
            if hedef_yatay.length() > 0:
                test_yonleri.append(hedef_yatay.normalized())
        
        en_iyi_yon = None
        en_iyi_skor = float('-inf')
        
        for yon in test_yonleri:
            # Bu yönde ne kadar güvenli?
            skor = 0.0
            
            # Tehlikeli nesnelerden uzaklık kontrolü
            for nesne in tehlikeli_nesneler:
                # Bu yönde ilerlersek nesneye ne kadar yaklaşırız?
                nesne_yonu = (nesne['pozisyon'] - self.rov.position).normalized()
                yon_nesne_aci = yon.dot(nesne_yonu)
                
                # Eğer bu yöne doğru gidersek nesneye yaklaşırsak, skor düşer
                if yon_nesne_aci > 0:  # Aynı yöne
                    uzaklik_skoru = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                    skor -= uzaklik_skoru * 2.0  # Tehlikeli nesneye yaklaşma cezası
                else:  # Uzaklaşma
                    skor += abs(yon_nesne_aci) * 1.0  # Uzaklaşma bonusu
            
            # Hedef yönüne yakınlık bonusu
            if hedef_vektoru.length() > 0:
                hedef_yatay = Vec3(hedef_vektoru.x, 0, hedef_vektoru.z)
                if hedef_yatay.length() > 0:
                    hedef_yatay = hedef_yatay.normalized()
                    hedef_benzerligi = yon.dot(hedef_yatay)
                    if hedef_benzerligi > 0:
                        skor += hedef_benzerligi * 0.5  # Hedefe yakınlık bonusu
            
            if skor > en_iyi_skor:
                en_iyi_skor = skor
                en_iyi_yon = yon
        
        # Eğer hiç güvenli yön bulunamazsa, en az tehlikeli olanı seç
        if en_iyi_yon is None:
            # Tüm tehlikeli nesnelerden uzaklaş
            uzaklasma_vektoru = Vec3(0, 0, 0)
            for nesne in tehlikeli_nesneler:
                uzaklasma_yonu = (self.rov.position - nesne['pozisyon']).normalized()
                uzaklasma_gucu = (kacinma_mesafesi - nesne['mesafe']) / kacinma_mesafesi
                uzaklasma_vektoru += uzaklasma_yonu * uzaklasma_gucu
            
            if uzaklasma_vektoru.length() > 0:
                return uzaklasma_vektoru.normalized()
            return Vec3(0, 1, 0)  # Son çare: yukarı
        
        return en_iyi_yon

# ==========================================
# 3. LİDER VE TAKİPÇİ (AI KONTROLLÜ)
# ==========================================
class LiderGNC(TemelGNC):
    def __init__(self, rov_entity, modem, filo_ref=None):
        super().__init__(rov_entity, modem)
        self.diger_lider_hedefi = None  # Yüzeydeki diğer liderin konumu
        self.diger_lider_bulundu = False  # Diğer lider bulundu mu?
        self.filo_ref = filo_ref  # Filo referansı (asıl hedef kontrolü için)
    
    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return 
        
        # ASIL LİDER KONTROLÜ: Asıl hedefe en yakın olan lider, diğer liderleri takip etmez
        asil_lider_mi = self._asil_lider_mi()
        
        # YÜZEYDE DİĞER LİDER BULMA: Sadece asıl lider değilse ve yüzeydeyse diğer lideri bul
        # Güvenlik kontrolü: MockROV için yuzeyde attribute'u olmayabilir
        rov_yuzeyde = getattr(self.rov, 'yuzeyde', self.rov.y >= 0 if hasattr(self.rov, 'y') else False)
        if not asil_lider_mi and rov_yuzeyde:
            diger_lider = self._yuzeydeki_diger_lider_bul()
            if diger_lider:
                if not self.diger_lider_bulundu:
                    # İlk kez bulundu (sadece bir kez mesaj yazdır)
                    self.diger_lider_bulundu = True
                    print(f"🌊 [LİDER] ROV-{self.rov.id} yüzeyde diğer lider ROV-{diger_lider.id} bulundu, ona doğru ilerliyor!")
                
                # Diğer liderin konumunu sürekli güncelle (hareket edebilir)
                self.diger_lider_hedefi = Vec3(diger_lider.x, 0.0, diger_lider.z)  # Yüzey seviyesi
            else:
                # Diğer lider bulunamadı, flag'i sıfırla (tekrar bulunabilir)
                if self.diger_lider_bulundu:
                    self.diger_lider_bulundu = False
        
        # Diğer lider hedefi varsa ona doğru ilerle (sadece asıl lider değilse)
        if not asil_lider_mi and self.diger_lider_hedefi is not None and rov_yuzeyde:
            # Önce asıl lidere mesafeyi kontrol et
            diger_lider = self._yuzeydeki_diger_lider_bul()
            if diger_lider:
                mesafe = distance(self.rov.position, diger_lider.position)
                iletisim_menzili = self.rov.sensor_config.get("iletisim_menzili", 35.0)
                
                # Eğer iletişim mesafesi kadar yaklaştıysa, takipçi ol ve bat
                if mesafe <= iletisim_menzili:
                    print(f"🌊 [LİDER→TAKİPÇİ] ROV-{self.rov.id} asıl lidere yaklaştı ({mesafe:.1f}m), takipçi olup batıyor!")
                    # Takipçi ol
                    self.rov.set("rol", 0)
                    # Filo sisteminde GNC tipini değiştir
                    if self.filo_ref:
                        # Bu ROV'un GNC sistemini bul
                        for i, gnc in enumerate(self.filo_ref.sistemler):
                            if gnc.rov.id == self.rov.id:
                                # Asıl liderin GNC sistemini bul ve modem referansını al
                                asil_lider_modem = None
                                for diger_gnc in self.filo_ref.sistemler:
                                    if diger_gnc.rov.id == diger_lider.id:
                                        asil_lider_modem = diger_gnc.modem if diger_gnc.modem else None
                                        break
                                
                                # GNC tipini takipçi yap
                                self.filo_ref._gnc_tipini_degistir(i, 'takipci', lider_modem_ref=asil_lider_modem)
                                
                                # Takipçi oldu, batma mantığını uygula
                                # GPS bilgilerinden derinlik al ve farklı derinlikte batır
                                lider_derinlik = diger_lider.y  # Lider ROV'un derinliği (y koordinatı)
                                
                                # Bu ROV için hedef derinlik: Lider derinliği - (ROV ID * 5) metre
                                # Böylece her ROV farklı derinlikte olur ve çarpışmaz
                                hedef_derinlik = lider_derinlik - (self.rov.id * 5.0)
                                # Minimum derinlik kontrolü (çok derine gitmesin)
                                hedef_derinlik = max(hedef_derinlik, -90.0)
                                
                                # Mevcut derinlik
                                mevcut_derinlik = self.rov.y
                                
                                # Derinlik farkına göre batırma hızı
                                derinlik_farki = hedef_derinlik - mevcut_derinlik
                                if derinlik_farki < -0.5:  # Hedef derinlik daha derinde
                                    # Batırma hızı: Derinlik farkına göre (maksimum -3.0)
                                    batirma_hizi = max(derinlik_farki * 0.5, -3.0)
                                    self.rov.velocity.y = batirma_hizi
                                elif self.rov.y >= 0:
                                    # Hala yüzeydeyse, hedef derinliğe doğru batır
                                    self.rov.velocity.y = -2.0
                                
                                break
                    
                    # Diğer lider hedefini sıfırla
                    self.diger_lider_hedefi = None
                    self.diger_lider_bulundu = False
                    return  # Takipçi oldu, normal lider mantığından çık
            
            mevcut = self.rov.position
            fark = self.diger_lider_hedefi - mevcut
            if fark.length() > 1.0:
                # Yüzeyde yatay hareket (y ekseni sabit)
                fark.y = 0
                if fark.length() > 0:
                    yon = fark.normalized()
                    self.vektor_to_motor(yon, guc_carpani=1.0)
                return
        elif not rov_yuzeyde:
            # Yüzeyden ayrıldıysa diğer lider hedefini sıfırla
            self.diger_lider_hedefi = None
            self.diger_lider_bulundu = False
        
        # Normal hedef takibi
        if self.hedef is None: return
        
        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, gelen uyarıyı görmezden gel (0 kabul et)
        if not self.ai_aktif:
            gat_kodu = 0
        
        mevcut = self.rov.position
        fark = self.hedef - mevcut
        
        # Hedefe ulaşma kontrolü: Yatay düzlemde (x, z) mesafesi kontrol et
        # Dikey (y) mesafesi farklı olabilir, bu yüzden sadece yatay mesafeye bak
        yatay_fark = Vec3(fark.x, 0, fark.z)
        if yatay_fark.length() < 0.5:  # Yatay düzlemde 0.5 birim yakınsa hedefe ulaşıldı
            return

        if self.hedef.y < 0: self.hedef.y = 0
        hedef_vektoru = fark.normalized()
        
        # BİRLEŞTİRİLMİŞ YAKINLAŞMA ÖNLEME VE GAT KODLARI
        kacinma_vektoru = self._yaklasma_onleme_vektoru(gat_kodu, hedef_vektoru)
        
        # GAT Tepkileri
        if gat_kodu == 1:  # ENGEL
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.3
                kacinma_vektoru = kacinma_vektoru.normalized()
            else:
                kacinma_vektoru = Vec3(1, 0, 0)  # Sağa
        elif gat_kodu == 2:  # CARPISMA
            # En uygun rota zaten hesaplandı
            pass
        elif gat_kodu == 3:  # KOPUK
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.2
                kacinma_vektoru = kacinma_vektoru.normalized()
            else:
                kacinma_vektoru = Vec3(0, 0.2, 0)
        
        # Vektör birleştirme
        if gat_kodu == 2:  # ÇARPISMA: En uygun rota direkt kullan
            yon = kacinma_vektoru if kacinma_vektoru.length() > 0 else Vec3(0, 0, 0)
        elif gat_kodu != 0:
            if kacinma_vektoru.length() > 0:
                yon = kacinma_vektoru * 0.8 + hedef_vektoru * 0.2
            else:
                yon = hedef_vektoru
        else:
            # Normal durum
            if kacinma_vektoru.length() > 0:
                yon = hedef_vektoru + kacinma_vektoru * 0.5
            else:
                yon = hedef_vektoru
        
        if yon.length() > 0:
            yon = yon.normalized()

        self.vektor_to_motor(yon)
    
    def _asil_lider_mi(self):
        """Bu lider asıl lider mi? (Asıl hedefe en yakın olan)"""
        if self.filo_ref is None or self.filo_ref.asil_hedef is None:
            return False
        
        # Asıl hedefe mesafeyi hesapla
        mesafe = distance(self.rov.position, self.filo_ref.asil_hedef)
        
        # Sadece aynı gruptaki liderler arasında asıl hedefe en yakın olanı bul
        # Farklı gruplardaki liderler birbirini etkilemez
        en_yakin_mesafe = float('inf')
        
        # Kendi grubundaki liderleri bul
        if hasattr(self.rov, 'environment_ref') and self.rov.environment_ref:
            iletisim_menzili = self.rov.sensor_config.get("iletisim_menzili", 35.0)
            
            for gnc in self.filo_ref.sistemler:
                if gnc.rov.role == 1 and gnc.rov.id != self.rov.id:
                    # İletişim kontrolü (aynı grupta mı?)
                    diger_rov = gnc.rov
                    diger_mesafe = distance(self.rov.position, diger_rov.position)
                    rov_yuzeyde = self.rov.y >= 0
                    diger_rov_yuzeyde = diger_rov.y >= 0
                    
                    iletisim_var = False
                    if rov_yuzeyde and diger_rov_yuzeyde:
                        iletisim_var = True
                    else:
                        iletisim_var = diger_mesafe < iletisim_menzili
                    
                    # Sadece aynı gruptaki liderlerle karşılaştır
                    if iletisim_var:
                        lider_mesafe = distance(diger_rov.position, self.filo_ref.asil_hedef)
                        if lider_mesafe < en_yakin_mesafe:
                            en_yakin_mesafe = lider_mesafe
        
        # Eğer aynı grupta başka lider yoksa, bu lider asıl liderdir
        if en_yakin_mesafe == float('inf'):
            return True
        
        # Bu lider aynı gruptaki liderler arasında asıl hedefe en yakın mı?
        return abs(mesafe - en_yakin_mesafe) < 0.1  # Küçük tolerans
    
    def _yuzeydeki_diger_lider_bul(self):
        """Yüzeydeki diğer lider ROV'u bulur (asıl lider hariç)"""
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return None
        
        for diger_rov in self.rov.environment_ref.rovs:
            if diger_rov.id == self.rov.id:
                continue
            if diger_rov.role == 1 and diger_rov.y >= 0:  # Lider ve yüzeyde
                # Asıl lider kontrolü: Sadece aynı gruptaki liderlerle karşılaştır
                if self.filo_ref and self.filo_ref.asil_hedef:
                    # İletişim kontrolü (aynı grupta mı?)
                    iletisim_menzili = self.rov.sensor_config.get("iletisim_menzili", 35.0)
                    mesafe = distance(self.rov.position, diger_rov.position)
                    # Yüzeyde oldukları için iletişim var
                    iletisim_var = True
                    
                    # Sadece aynı gruptaki liderlerle karşılaştır
                    if iletisim_var:
                        diger_lider_mesafe = distance(diger_rov.position, self.filo_ref.asil_hedef)
                        bu_lider_mesafe = distance(self.rov.position, self.filo_ref.asil_hedef)
                        # Eğer diğer lider asıl hedefe daha yakınsa, onu takip etme
                        if diger_lider_mesafe < bu_lider_mesafe:
                            continue
                return diger_rov
        return None

class TakipciGNC(TemelGNC):
    def __init__(self, rov_entity, modem, lider_modem_ref=None):
        super().__init__(rov_entity, modem)
        self.lider_ref = lider_modem_ref
        self.iletisim_kopma_sayaci = 0  # İletişim kopma sayacı (gecikme için)

    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return
        
        # İLETİŞİM KOPMA KONTROLÜ: Liderle iletişim yoksa otomatik lider ol
        # ÖNEMLİ: Sadece kendi grubunda lider yoksa lider ol
        # ÖNEMLİ: Daha uzun süre beklemek için sayacı artırdık (geçici kopmaları önlemek için)
        if not self.rov.lider_ile_iletisim:
            self.iletisim_kopma_sayaci += 1
            # 50 frame (yaklaşık 1.5 saniye) iletişim yoksa lider ol
            # Bu, çarpışma önleme mekanizmasının neden olduğu geçici iletişim kopmalarını önler
            if self.iletisim_kopma_sayaci >= 50:
                # Kendi grubunda lider var mı kontrol et
                grup_lideri_var = self._kendi_grubunda_lider_var_mi()
                if not grup_lideri_var:
                    self._lider_ol()
                else:
                    # Grubunda lider var, sayacı sıfırla
                    self.iletisim_kopma_sayaci = 0
                return
        else:
            # İletişim varsa, sayacı yavaşça azalt (histerezis için)
            if self.iletisim_kopma_sayaci > 0:
                self.iletisim_kopma_sayaci = max(0, self.iletisim_kopma_sayaci - 2)
        
        # YÜZEY İLETİŞİMİ: Yüzeydeyse ve başka yüzeydeki lider varsa ona doğru ilerle
        # Güvenlik kontrolü: MockROV için yuzeyde attribute'u olmayabilir
        rov_yuzeyde = getattr(self.rov, 'yuzeyde', self.rov.y >= 0 if hasattr(self.rov, 'y') else False)
        if rov_yuzeyde:
            yuzeydeki_lider = self._yuzeydeki_lider_bul()
            if yuzeydeki_lider:
                # Yüzeydeki liderin konumuna doğru ilerle
                lider_pozisyon = yuzeydeki_lider.position
                fark = lider_pozisyon - self.rov.position
                if fark.length() > 1.0:
                    hedef_vektoru = fark.normalized()
                    # Yüzeyde yatay hareket (y ekseni sabit)
                    hedef_vektoru.y = 0
                    if hedef_vektoru.length() > 0:
                        hedef_vektoru = hedef_vektoru.normalized()
                        self.vektor_to_motor(hedef_vektoru, guc_carpani=1.0)
                return
        
        # YENİDEN BAĞLANMA: Yüzeydeyse ve sonar mesafesi içinde başka ROV varsa takipçi ol ve bat
        if rov_yuzeyde:
            yakin_rov = self._sonar_mesafesinde_rov_bul()
            if yakin_rov and yakin_rov.role == 1:  # Lider ROV bulundu
                # Takipçi ol
                self.rov.set("rol", 0)
                
                # GPS bilgilerinden derinlik al ve farklı derinlikte batır
                lider_derinlik = yakin_rov.y  # Lider ROV'un derinliği (y koordinatı)
                
                # Bu ROV için hedef derinlik: Lider derinliği - (ROV ID * 5) metre
                # Böylece her ROV farklı derinlikte olur ve çarpışmaz
                hedef_derinlik = lider_derinlik - (self.rov.id * 5.0)
                # Minimum derinlik kontrolü (çok derine gitmesin)
                hedef_derinlik = max(hedef_derinlik, -90.0)
                
                # Mevcut derinlik
                mevcut_derinlik = self.rov.y
                
                # Derinlik farkına göre batırma hızı
                derinlik_farki = hedef_derinlik - mevcut_derinlik
                if derinlik_farki < -0.5:  # Hedef derinlik daha derinde
                    # Batırma hızı: Derinlik farkına göre (maksimum -3.0)
                    batirma_hizi = max(derinlik_farki * 0.5, -3.0)
                    self.rov.velocity.y = batirma_hizi
                elif self.rov.y >= 0:
                    # Hala yüzeydeyse, hedef derinliğe doğru batır
                    self.rov.velocity.y = -2.0
                return
        
        if self.hedef is None: return

        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, tehlike yokmuş gibi (0) davran
        if not self.ai_aktif:
            gat_kodu = 0

        fark = self.hedef - self.rov.position
        
        # Hedefe ulaşma kontrolü: Yatay düzlemde (x, z) mesafesi kontrol et
        # Dikey (y) mesafesi farklı olabilir, bu yüzden sadece yatay mesafeye bak
        yatay_fark = Vec3(fark.x, 0, fark.z)
        if yatay_fark.length() < 0.5:  # Yatay düzlemde 0.5 birim yakınsa hedefe ulaşıldı
            return
        
        hedef_vektoru = fark.normalized()
        
        # BİRLEŞTİRİLMİŞ YAKINLAŞMA ÖNLEME VE GAT KODLARI
        # GAT kodlarına göre en uygun kaçınma vektörünü hesapla
        kacinma_vektoru = self._yaklasma_onleme_vektoru(gat_kodu, hedef_vektoru)
        
        # GAT Tepkileri (yakınlaşma önleme ile birleştirilmiş)
        if gat_kodu == 1:  # ENGEL
            # Yakınlaşma önleme zaten hesaplandı, sadece yukarı bileşen ekle
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.3  # Biraz yukarı
                kacinma_vektoru = kacinma_vektoru.normalized()
            else:
                kacinma_vektoru = Vec3(0, 1.0, 0) + (hedef_vektoru * -0.5)
        elif gat_kodu == 2:  # CARPISMA
            # En uygun rota zaten hesaplandı (_en_uygun_rota_bul)
            # Ek işlem gerekmez
            pass
        elif gat_kodu == 3:  # KOPUK
            # Biraz yukarı çık
            if kacinma_vektoru.length() > 0:
                kacinma_vektoru.y += 0.2
                kacinma_vektoru = kacinma_vektoru.normalized()
            else:
                kacinma_vektoru = Vec3(0, 0.2, 0)
        elif gat_kodu == 5:  # UZAK
            # Normal hareket, kaçınma yok
            pass

        # Vektör Birleştirme
        if gat_kodu == 2:  # ÇARPISMA: En uygun rota direkt kullan
            nihai_vektor = kacinma_vektoru
        elif gat_kodu != 0 and gat_kodu != 5:
            # Kaçınma vektörü + hedef vektörü (kaçınma öncelikli)
            if kacinma_vektoru.length() > 0:
                nihai_vektor = kacinma_vektoru * 0.8 + hedef_vektoru * 0.2
            else:
                nihai_vektor = kacinma_vektoru + (hedef_vektoru * 0.1)
        else:
            # Normal durum: Kaçınma varsa ekle, yoksa sadece hedef
            if kacinma_vektoru.length() > 0:
                nihai_vektor = hedef_vektoru + kacinma_vektoru * 0.5
            else:
                nihai_vektor = hedef_vektoru
        
        # Normalize et
        if nihai_vektor.length() > 0:
            nihai_vektor = nihai_vektor.normalized()

        guc = 1.0
        if gat_kodu == 5: guc = 1.5 
        if gat_kodu == 1: guc = 0.5 
        
        self.vektor_to_motor(nihai_vektor, guc_carpani=guc)
    
    def _kendi_grubunda_lider_var_mi(self):
        """Kendi grubunda (iletişim bağlantılı) lider var mı kontrol eder"""
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return False
        
        iletisim_menzili = self.rov.sensor_config.get("iletisim_menzili", 35.0)
        
        for diger_rov in self.rov.environment_ref.rovs:
            if diger_rov.id == self.rov.id:
                continue
            
            mesafe = distance(self.rov.position, diger_rov.position)
            rov_yuzeyde = self.rov.y >= 0
            diger_rov_yuzeyde = diger_rov.y >= 0
            
            iletisim_var = False
            if rov_yuzeyde and diger_rov_yuzeyde:
                iletisim_var = True
            else:
                iletisim_var = mesafe < iletisim_menzili
            
            # İletişim kurabildiği bir ROV lider ise, kendi grubunda lider var
            if iletisim_var and diger_rov.role == 1:
                return True
        
        return False
    
    def _lider_ol(self):
        """Takipçi ROV'u lider yapar (iletişim koptuğunda)"""
        if self.rov.role == 1:  # Zaten lider
            return
        
        # Kendi grubunda lider yoksa, bu ROV lider ol
        # Sonsuz döngü önleme: Eğer bu ROV son zamanlarda çok sık lider/takipçi değiştirdiyse, lider olma
        rov_id = self.rov.id
        # Filo referansını bul (ROV'dan veya environment_ref'den)
        filo_ref = None
        if hasattr(self.rov, 'filo_ref'):
            filo_ref = self.rov.filo_ref
        elif hasattr(self.rov, 'environment_ref') and self.rov.environment_ref:
            if hasattr(self.rov.environment_ref, 'filo'):
                filo_ref = self.rov.environment_ref.filo
        
        if filo_ref and rov_id in filo_ref.lider_degisim_sayaci and filo_ref.lider_degisim_sayaci[rov_id] > 3:
            # Bu ROV çok sık değişiyor, lider olma (sonsuz döngü önleme)
            return
        
        print(f"🔴 [İLETİŞİM KOPMA] ROV-{self.rov.id} liderle iletişim koptu, otomatik lider oldu!")
        self.rov.set("rol", 1)
        
        # Lider değişim sayacını artır
        if filo_ref:
            if rov_id not in filo_ref.lider_degisim_sayaci:
                filo_ref.lider_degisim_sayaci[rov_id] = 0
            filo_ref.lider_degisim_sayaci[rov_id] += 0.5
    
    def _yuzeydeki_lider_bul(self):
        """Yüzeydeki lider ROV'u bulur"""
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return None
        
        for diger_rov in self.rov.environment_ref.rovs:
            if diger_rov.id == self.rov.id:
                continue
            if diger_rov.role == 1 and diger_rov.y >= 0:  # Lider ve yüzeyde
                return diger_rov
        return None
    
    def _sonar_mesafesinde_rov_bul(self):
        """Sonar mesafesi (engel_mesafesi) içinde başka ROV var mı kontrol eder"""
        if not hasattr(self.rov, 'environment_ref') or not self.rov.environment_ref:
            return None
        
        sonar_mesafesi = self.rov.sensor_config.get("engel_mesafesi", 20.0)
        
        for diger_rov in self.rov.environment_ref.rovs:
            if diger_rov.id == self.rov.id:
                continue
            mesafe = distance(self.rov.position, diger_rov.position)
            if mesafe <= sonar_mesafesi:
                return diger_rov
        return None
