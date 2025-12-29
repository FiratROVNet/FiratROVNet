from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from FiratROVNet.config import Formasyon
from ursina import *
import numpy as np
import os

# 1. KURULUM
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
app.sim_olustur(n_rovs=6, n_engels=15)

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ Model yüklenemedi, AI devre dışı."); 
    beyin = None

# Filo sistemini otomatik kurulum ile oluştur
# (otomatik_kurulum içinde rol ataması yapılıyor, manuel atama gerekmez)
filo = Filo()
formasyon = Formasyon(filo)

tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,
    ortam_ref=app,  # Ortam referansını ekle (hedef görselleştirme için)
    baslangic_hedefleri={
        0: (150, 10, 0)    # Lider: (x, y, z)
        # Takipçiler için hedef belirtilmezse hedef atanmaz
    }
    # İsteğe bağlı parametreler (yukarıdaki satıra virgül ekleyerek kullanın):
    
    # Örnek 1: Özel sensör ayarları (tüm ROV'lar için ortak)
    # sensor_ayarlari={
    #     'engel_mesafesi': 25.0,
    #     'iletisim_menzili': 40.0,
    #     'min_pil_uyarisi': 15.0
    # }
    
    # Örnek 2: Lider ve takipçi için ayrı sensör ayarları (Varsayılan olarak otomatik uygulanır)
    # sensor_ayarlari={
    #     'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
    #     'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
    # }
    
    # Örnek 3: Her ROV için özel sensör ayarları
    # sensor_ayarlari={
    #     0: {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0},  # Lider
    #     1: {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0},  # Takipçi 1
    #     2: {'engel_mesafesi': 20.0, 'iletisim_menzili': 35.0}   # Takipçi 2
    # }
    
    # Örnek 4: Özel modem ayarları ile
    # modem_ayarlari={
    #     'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.1, 'gecikme': 0.5},
    #     'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1, 'gecikme': 0.5}
    # }
    
    # Örnek 5: Tüm parametrelerle tam kontrol
    # modem_ayarlari={
    #     'lider': {'gurultu_orani': 0.03, 'kayip_orani': 0.05, 'gecikme': 0.4},
    #     'takipci': {'gurultu_orani': 0.12, 'kayip_orani': 0.15, 'gecikme': 0.5}
    # },
    # sensor_ayarlari={
    #     'lider': {'engel_mesafesi': 30.0, 'iletisim_menzili': 50.0, 'min_pil_uyarisi': 20.0},
    #     'takipci': {'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0, 'min_pil_uyarisi': 15.0}
    # }
)
# Filo referansını Ortam'a ekle
app.filo = filo

# Konsola git ve move fonksiyonlarını ekle (wrapper ile güvenli çağrı)
def git_wrapper(rov_id, x, z, y=None, ai=True):
    """
    Konsol için git fonksiyonu wrapper'ı.
    Kullanım: git(0, 40, 60, 0) veya git(0, 40, 60, 0, ai=True)
    """
    if filo is None:
        print("❌ [HATA] Filo henüz oluşturulmamış!")
        return
    return filo.git(rov_id, x, z, y, ai)

def move_wrapper(rov_id, yon, guc=1.0):
    """
    Konsol için move fonksiyonu wrapper'ı.
    Kullanım: move(0, 'ileri', 1.0) veya move(1, 'sag', 0.5)
    """
    if filo is None:
        print("❌ [HATA] Filo henüz oluşturulmamış!")
        return
    return filo.move(rov_id, yon, guc)

def get_wrapper(rov_id, veri_tipi):
    """
    Konsol için get fonksiyonu wrapper'ı.
    Kullanım: get(0, 'gps') veya get(1, 'batarya')
    """
    if filo is None:
        print("❌ [HATA] Filo henüz oluşturulmamış!")
        return None
    return filo.get(rov_id, veri_tipi)

def set_wrapper(rov_id, ayar_adi, deger):
    """
    Konsol için set fonksiyonu wrapper'ı.
    Kullanım: set(0, 'rol', 1) veya set(1, 'engel_mesafesi', 30.0)
    """
    if filo is None:
        print("❌ [HATA] Filo henüz oluşturulmamış!")
        return
    return filo.set(rov_id, ayar_adi, deger)

app.konsola_ekle("git", git_wrapper)
app.konsola_ekle("move", move_wrapper)
app.konsola_ekle("get", get_wrapper)
app.konsola_ekle("set", set_wrapper)
app.konsola_ekle("gnc", filo.sistemler)
app.konsola_ekle("filo", filo)  # Filo nesnesini konsola ekle
app.konsola_ekle("rovs", app.rovs)
app.konsola_ekle("cfg", cfg)
app.konsola_ekle("harita", app.harita)  # Harita nesnesini konsola ekle

# Ada ve ROV konum yönetimi için wrapper'lar
def Ada_wrapper(ada_id, x=None, y=None):
    """
    Konsol için Ada fonksiyonu wrapper'ı.
    Kullanım: Ada(0, 50, 60) veya Ada(0)  # Konum almak için
    """
    if app is None:
        print("❌ [HATA] Ortam henüz oluşturulmamış!")
        return None
    return app.Ada(ada_id, x, y)

def ROV_wrapper(rov_id, x=None, y=None, z=None):
    """
    Konsol için ROV fonksiyonu wrapper'ı.
    Kullanım: ROV(0, 10, -5, 20) veya ROV(0)  # Konum almak için
    """
    if app is None:
        print("❌ [HATA] Ortam henüz oluşturulmamış!")
        return None
    return app.ROV(rov_id, x, y, z)




app.konsola_ekle("Ada", Ada_wrapper)
app.konsola_ekle("ROV", ROV_wrapper)
# Formasyon sınıfını konsola ekle (sınıf metodlarına erişim için)
app.konsola_ekle("Formasyon", formasyon)

print("✅ Sistem aktif.")
print("🗺️  Harita aktif! Kullanım: harita.ekle(x_2d, y_2d)")
print("🏝️  Ada yönetimi aktif! Kullanım: Ada(0, 50, 60) - Ada 0'ı (50, 60) pozisyonuna taşı")
print("🤖 ROV yönetimi aktif! Kullanım: ROV(0, 10, -5, 20) - ROV 0'ı (10, -5, 20) pozisyonuna taşı")


# 2. ANA DÖNGÜ
def update():
    try:
        # Thread-safe komut kuyruğunu işle (konsoldan gelen komutlar için)
        filo.execute_queued_commands()
        
        veri = app.simden_veriye()
        
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        if ai_aktif and beyin:
            try: 
                tahminler, _, _ = beyin.analiz_et(veri)
            except: 
                tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)

        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
        
        for i, gat_kodu in enumerate(tahminler):
            # GAT kodunu ROV'a kaydet
            app.rovs[i].gat_kodu = gat_kodu
            
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                # GAT koduna göre renk değiştir (FBX model için de çalışır)
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            # FBX model kullanılıyorsa, GAT kodunu görünür kılmak için color'ı blend et
            # (FBX model texture kullanıyorsa color değişimi daha az görünür olabilir)
            if hasattr(app.rovs[i], 'model') and isinstance(app.rovs[i].model, str) and app.rovs[i].model.endswith('.fbx'):
                # FBX model için color'ı daha belirgin yapmak için alpha veya tint kullan
                # Ursina'da color direkt olarak texture ile blend edilir
                pass  # Color zaten ayarlandı, Ursina otomatik blend eder
            
            # Label scale'ini büyüt (uzaktan okunabilir) - GAT kodu için daha büyük
            app.rovs[i].label.scale = 6000  # Sabit büyük scale
            app.rovs[i].label.y = 300  # Y eksenini artır (ROV'un üstünde daha yüksekte)
            app.rovs[i].label.color = app.rovs[i].color 
            app.rovs[i].label.background = False  # Arka plan ekle (daha görünür)
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            # GAT kodunu label'da büyük ve görünür şekilde göster
            # gat_kodu bir integer, liste indexi olarak kullanılmalı
            gat_kodu = app.rovs[i].gat_kodu
            if 0 <= gat_kodu < len(durum_txts):
                app.rovs[i].label.text = durum_txts[gat_kodu]+str(i)
            else:
                app.rovs[i].label.text = f"GAT:{gat_kodu}+{str(i)}"
        
        filo.guncelle_hepsi(tahminler)
        
        # Harita güncelle (Matplotlib penceresi) - Throttled içeride yapılıyor
        if hasattr(app, 'harita') and app.harita is not None:
            try:
                # Matplotlib penceresini güncelle (throttled, non-blocking)
                app.harita.update()
                # plt.pause() kaldırıldı - harita.update() içinde throttle var
            except Exception as e:
                # Harita güncelleme hatası (sessizce geç, simülasyon devam etsin)
                pass
        
    except Exception as e: 
        pass

app.set_update_function(update)
# Input handler override edilmedi - Ursina'nın varsayılan input handler'ı çalışıyor
# EditorCamera'nın P tuşu ve diğer kontrolleri çalışacak

# 4. ÇALIŞTIRMA
if __name__ == "__main__":
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        pass
    finally: 
        os.system('stty sane')
        os._exit(0)
