from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import os

# 1. KURULUM
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
# Hedef nokta belirle (engeller bu noktadan uzak olacak)
hedef_nokta = Vec3(40, 0, 60)
app.sim_olustur(n_rovs=4, n_engels=15, hedef_nokta=hedef_nokta)

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ Model yüklenemedi, AI devre dışı."); 
    beyin = None

# Filo sistemini otomatik kurulum ile oluştur
filo = Filo()
tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,
    lider_id=0,
    baslangic_hedefleri={
        0: (40, 0, 60),    # Lider: (x, y, z)
        1: (35, -10, 50),  # Takipçi 1
        2: (40, -10, 50),  # Takipçi 2
        3: (45, -10, 50)   # Takipçi 3
    }
    # İsteğe bağlı parametreler (yukarıdaki satıra virgül ekleyerek kullanın):
    
    # Örnek 1: Özel sensör ayarları (tüm ROV'lar için ortak)
    # sensor_ayarlari={
    #     'engel_mesafesi': 25.0,
    #     'iletisim_menzili': 40.0,
    #     'min_pil_uyarisi': 15.0
    # }
    
    # Örnek 2: Lider ve takipçi için ayrı sensör ayarları
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
app.konsola_ekle("git", filo.git)
app.konsola_ekle("gnc", filo.sistemler)
app.konsola_ekle("rovs", app.rovs)
app.konsola_ekle("cfg", cfg)
print("✅ Sistem aktif.")


# 2. ANA DÖNGÜ
def update():
    try:
        # Simülasyondan GAT verisi al (parametrelerle özelleştirilebilir)
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
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}{ek}"
        
        filo.guncelle_hepsi(tahminler)
        
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
