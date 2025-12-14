import numpy as np
from ursina import Vec3, time
from .config import cfg
from .iletisim import AkustikModem
import math

# ==========================================
# 1. FİLO (ROV FİLO YÖNETİCİSİ)
# ==========================================
class Filo:
    def __init__(self):
        self.sistemler = [] 

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
            sensor_ayarlari (dict, optional): Sensör ayarları. İki format desteklenir:
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
                    0: {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0},
                    1: {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0},
                    2: {'engel_mesafesi': 20.0, 'iletisim_menzili': 35.0}
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
            
            # Farklı lider ID ile
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                lider_id=2,  # ROV-2 lider olacak
                baslangic_hedefleri={
                    2: (50, 0, 70),    # Lider (ROV-2)
                    0: (30, -10, 50),  # Takipçi
                    1: (35, -10, 50),  # Takipçi
                    3: (40, -10, 50)   # Takipçi
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
            
            # Sensör ayarları ile (ortak ayarlar)
            tum_modemler = filo.otomatik_kurulum(
                rovs=app.rovs,
                sensor_ayarlari={
                    'engel_mesafesi': 25.0,
                    'iletisim_menzili': 40.0,
                    'min_pil_uyarisi': 15.0
                }
            )
        """
        # Varsayılan modem ayarları
        if modem_ayarlari is None:
            modem_ayarlari = {
                'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.1, 'gecikme': 0.5},
                'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1, 'gecikme': 0.5}
            }
        
        # Varsayılan sensör ayarları
        varsayilan_sensor_ayarlari = {
            'engel_mesafesi': 20.0,
            'iletisim_menzili': 35.0,
            'min_pil_uyarisi': 10.0
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
                
                # LiderGNC oluştur ve ekle
                gnc = LiderGNC(rov, lider_modem)
                self.ekle(gnc)
                
                # Başlangıç hedefi varsa ata
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    self.git(i, hedef[0], hedef[2], hedef[1] if len(hedef) > 2 else None)
                elif baslangic_hedefleri is None:
                    # Varsayılan lider hedefi
                    self.git(i, 40, 60, 0)
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
                
                # Başlangıç hedefi varsa ata
                if baslangic_hedefleri and i in baslangic_hedefleri:
                    hedef = baslangic_hedefleri[i]
                    self.git(i, hedef[0], hedef[2], hedef[1] if len(hedef) > 2 else None)
                elif baslangic_hedefleri is None:
                    # Varsayılan takipçi hedefi (formasyon)
                    offset_x = 30 + (i * 5)
                    self.git(i, offset_x, 50, -10)
        
        # Rehberi dağıt
        self.rehber_dagit(tum_modemler)
        
        print(f"✅ GNC Sistemi Kuruldu: {len(rovs)} ROV (Lider: ROV-{lider_id})")
        
        return tum_modemler

    def guncelle_hepsi(self, tahminler):
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])

    # --- GÜNCELLENEN GİT FONKSİYONU ---
    def git(self, rov_id, x, z, y=None, ai=True):
        """
        KONSOL KOMUTU: Hedefe git.
        Parametreler: (ID, X, Z, Y=Derinlik, ai=True/False)
        """
        if 0 <= rov_id < len(self.sistemler):
            # Manuel modu kapat, otopilotu aç
            self.sistemler[rov_id].manuel_kontrol = False
            
            # AI Durumunu Ayarla
            self.sistemler[rov_id].ai_aktif = ai
            
            # Hedef Ata
            hedef_y = y if y is not None else self.sistemler[rov_id].rov.y
            
            ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
            print(f"🔵 [KOMUTAN] ROV-{rov_id} Rota: {x}, {z}, {hedef_y} | AI: {ai_durum}")
            
            self.sistemler[rov_id].hedef_atama(x, hedef_y, z)
        else:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id}")

    def set(self, rov_id, ayar_adi, deger):
        """
        ROV ayarlarını değiştirir.
        
        Args:
            rov_id: ROV ID
            ayar_adi: Ayar adı ('rol', 'renk', 'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi')
            deger: Ayar değeri
        
        Örnekler:
            filo.set(0, 'rol', 1)  # ROV-0'ı lider yap
            filo.set(1, 'renk', (255, 0, 0))  # ROV-1'i kırmızı yap
            filo.set(2, 'engel_mesafesi', 30.0)  # ROV-2'nin engel mesafesini ayarla
        """
        if 0 <= rov_id < len(self.sistemler):
            rov = self.sistemler[rov_id].rov
            rov.set(ayar_adi, deger)
            print(f"✅ [FİLO] ROV-{rov_id} ayarı güncellendi: {ayar_adi} = {deger}")
        else:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id}")

    def get(self, rov_id, veri_tipi):
        """
        ROV bilgilerini alır.
        
        Args:
            rov_id: ROV ID
            veri_tipi: Veri tipi ('gps', 'hiz', 'batarya', 'rol', 'renk', 'sensör', 
                                  'engel_mesafesi', 'iletisim_menzili', 'min_pil_uyarisi', 'sonar')
        
        Returns:
            İstenen veri tipine göre değer
        
        Örnekler:
            pozisyon = filo.get(0, 'gps')
            rol = filo.get(1, 'rol')
            sensörler = filo.get(2, 'sensör')
        """
        if 0 <= rov_id < len(self.sistemler):
            rov = self.sistemler[rov_id].rov
            return rov.get(veri_tipi)
        else:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id}")
            return None

    def move(self, rov_id, yon, birim=1.0):
        """
        ROV'a bir birimlik hareket verir.
        
        Args:
            rov_id: ROV ID
            yon: Hareket yönü ('ileri', 'geri', 'sag', 'sol', 'cik', 'bat')
            birim: Hareket birimi (varsayılan: 1.0)
        
        Örnekler:
            filo.move(0, 'ileri')  # ROV-0 bir birim ileri
            filo.move(1, 'sag', 2.0)  # ROV-1 iki birim sağa
            filo.move(2, 'cik')  # ROV-2 bir birim yukarı
        """
        if 0 <= rov_id < len(self.sistemler):
            rov = self.sistemler[rov_id].rov
            # Havuz sınır kontrolü yapılacak
            if rov.environment_ref:
                havuz_genisligi = getattr(rov.environment_ref, 'havuz_genisligi', 200)
                havuz_yari_genislik = havuz_genisligi / 2
                
                # Mevcut pozisyon
                yeni_x, yeni_y, yeni_z = rov.x, rov.y, rov.z
                
                # Hareket vektörü
                hareket_miktari = birim * 1.0
                
                if yon == 'ileri':
                    yeni_z += hareket_miktari
                elif yon == 'geri':
                    yeni_z -= hareket_miktari
                elif yon == 'sag':
                    yeni_x += hareket_miktari
                elif yon == 'sol':
                    yeni_x -= hareket_miktari
                elif yon == 'cik':
                    yeni_y += hareket_miktari
                elif yon == 'bat':
                    if rov.role != 1:  # Lider batırılamaz
                        yeni_y -= hareket_miktari
                    else:
                        print(f"⚠️ [FİLO] ROV-{rov_id} lider, batırılamaz!")
                        return
                
                # Havuz sınır kontrolü
                if abs(yeni_x) > havuz_yari_genislik:
                    yeni_x = np.sign(yeni_x) * havuz_yari_genislik
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırına ulaştı (X)")
                
                if abs(yeni_z) > havuz_yari_genislik:
                    yeni_z = np.sign(yeni_z) * havuz_yari_genislik
                    print(f"⚠️ [FİLO] ROV-{rov_id} havuz sınırına ulaştı (Z)")
                
                # Y ekseni kontrolü (su yüzeyi ve deniz tabanı)
                if yeni_y > 0.5:
                    yeni_y = 0.5
                if yeni_y < -100:
                    yeni_y = -100
                
                # Pozisyonu güncelle
                rov.position = Vec3(yeni_x, yeni_y, yeni_z)
                print(f"✅ [FİLO] ROV-{rov_id} {yon} yönünde {birim} birim hareket etti")
            else:
                # Environment referansı yoksa direkt move komutu kullan
                rov.move(yon, birim)
        else:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id}")

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

        guc = self.hiz_limiti * guc_carpani

        if vektor.x > 0.1: self.rov.move("sag", abs(vektor.x) * guc)
        elif vektor.x < -0.1: self.rov.move("sol", abs(vektor.x) * guc)

        if vektor.y > 0.1: self.rov.move("cik", abs(vektor.y) * guc)
        elif vektor.y < -0.1:
            # Lider ROV batırılamaz
            if self.rov.role != 1:
                self.rov.move("bat", abs(vektor.y) * guc)
            # Lider için bat komutu yok sayılır

        if vektor.z > 0.1: self.rov.move("ileri", abs(vektor.z) * guc)
        elif vektor.z < -0.1: self.rov.move("geri", abs(vektor.z) * guc)

# ==========================================
# 3. LİDER VE TAKİPÇİ (AI KONTROLLÜ)
# ==========================================
class LiderGNC(TemelGNC):
    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return 
        if self.hedef is None: return
        
        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, gelen uyarıyı görmezden gel (0 kabul et)
        if not self.ai_aktif:
            gat_kodu = 0
        
        mevcut = self.rov.position
        fark = self.hedef - mevcut
        if fark.length() < 1.0: return

        # Lider için hedef her zaman su yüzeyinde (y >= 0)
        if self.hedef.y < 0: self.hedef.y = 0
        yon = fark.normalized()
        
        # Lider için aşağı yön bileşenini kaldır (batırılamaz)
        if yon.y < 0:
            yon.y = 0
            if yon.length() > 0:
                yon = yon.normalized()

        if gat_kodu == 1: yon += Vec3(1, 0, 0) 
        elif gat_kodu == 2: yon = Vec3(0, 0, 0)

        self.vektor_to_motor(yon)

class TakipciGNC(TemelGNC):
    def __init__(self, rov_entity, modem, lider_modem_ref=None):
        super().__init__(rov_entity, modem)
        self.lider_ref = lider_modem_ref

    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return
        if self.hedef is None: return

        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, tehlike yokmuş gibi (0) davran
        if not self.ai_aktif:
            gat_kodu = 0

        fark = self.hedef - self.rov.position
        if fark.length() < 1.5: return
        
        hedef_vektoru = fark.normalized()
        kacinma_vektoru = Vec3(0,0,0)

        # GAT Tepkileri
        if gat_kodu == 1: 
            kacinma_vektoru = Vec3(0, 1.0, 0) + (hedef_vektoru * -0.5)
        elif gat_kodu == 2: 
            kacinma_vektoru = -hedef_vektoru * 1.5
        elif gat_kodu == 3: 
            kacinma_vektoru = Vec3(0, 0.2, 0) 
        elif gat_kodu == 5: 
            pass

        # Vektör Birleştirme
        if gat_kodu != 0 and gat_kodu != 5: 
            nihai_vektor = kacinma_vektoru + (hedef_vektoru * 0.1)
        else:
            nihai_vektor = hedef_vektoru

        guc = 1.0
        if gat_kodu == 5: guc = 1.5 
        if gat_kodu == 1: guc = 0.5 
        
        self.vektor_to_motor(nihai_vektor, guc_carpani=guc)
