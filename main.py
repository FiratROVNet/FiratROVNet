from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo, Debug
from GAT.gat_test import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import os

# 1. KURULUM
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
# rov_model: 'bluerov2' (varsayılan), 'submarine'
app.sim_olustur(n_rovs=6, n_islands=6, havuz_genisligi=200, rov_model='submarine')

# GAT Modeli Yükleme
try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
    print("✅ GAT modeli yüklendi.")
except Exception as e: 
    print(f"⚠️ Model yüklenemedi, AI devre dışı: {e}")
    beyin = None

# Filo sistemini otomatik kurulum ile oluştur
filo = Filo()
filo.otomatik_kurulum(
    rovs=app.rovs,
    ortam_ref=app,
    baslangic_hedefleri={
        0: (150, 10, 0)  # Lider: (x, y, z)
    }
)
app.filo = filo

# Debug sınıfı (APF/GNC fonksiyonları: debug.list(), debug.apf(0), debug.apf() ile kullanım)
debug = Debug(filo)

# Minimap otomatik açık (ölçek 1.0)
filo.minimap(scale=1.0)

# Konsol fonksiyonları (interaktif Python konsolu için)
app.konsola_ekle("git", lambda rov_id, x, z, y=None, ai=True: filo.git(rov_id, x, z, y, ai))
app.konsola_ekle("move", lambda rov_id, yon, guc=1.0: filo.move(rov_id, yon, guc))
app.konsola_ekle("get", lambda rov_id, veri_tipi: filo.get(rov_id, veri_tipi))
app.konsola_ekle("set", lambda rov_id, ayar_adi, deger: filo.set(rov_id, ayar_adi, deger))
app.konsola_ekle("Ada", lambda ada_id, x=None, y=None: app.Ada(ada_id, x, y))
app.konsola_ekle("ROV", lambda rov_id, x=None, y=None, z=None: app.ROV(rov_id, x, y, z))
app.konsola_ekle("filo", filo)
app.konsola_ekle("rovs", app.rovs)
app.konsola_ekle("cfg", cfg)
app.konsola_ekle("debug", debug)

print("✅ Sistem aktif.")
print("🗺️  Harita aktif! Kullanım: harita.ekle(x_2d, y_2d)")
print("🏝️  Ada yönetimi aktif! Kullanım: Ada(0, 50, 60)")
print("🤖 ROV yönetimi aktif! Kullanım: ROV(0, 10, -5, 20)")
print("🔧 Debug aktif! Kullanım: debug.list(), debug.apf(0), debug.apf() ile kullanım bilgisi")


# 2. ANA DÖNGÜ
def update():
    """Ana simülasyon döngüsü - GAT kodlarını hesaplar ve ROV'ları günceller."""
    try:
        # Sonar iletişim çizgilerini güncelle
        app.guncelle_sonar_cizgileri()
        
        # Önce kuyruktaki komutları işle (git vb.) — hedef ataması güncellemeden önce yapılsın
        if hasattr(app, 'filo') and app.filo is not None:
            try:
                app.filo.execute_queued_commands()
            except Exception:
                pass
        # Simülasyon verilerini al
        veri = app.simden_veriye()
        
        # GAT tahminleri hesapla
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        if ai_aktif and beyin:
            try: 
                tahminler, _, _ = beyin.analiz_et(veri)
            except Exception as e:
                print(f"⚠️ GAT analiz hatası: {e}")
                tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)
        #print(tahminler)
        filo.guncelle_hepsi(tahminler)

        # GAT kodlarına göre görselleştirme
        # Kod 0: OK (turuncu), Kod 1: ENGEL (kırmızı), Kod 2: CARPISMA (siyah), 
        # Kod 3: KOPUK (sarı), Kod 4: UZAK (magenta)
        kod_renkleri = {
            0: color.orange,   # OK
            1: color.red,      # ENGEL
            2: color.black,    # CARPISMA
            3: color.yellow,   # KOPUK
            4: color.magenta   # UZAK
        }
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "UZAK"]
        
        # Her ROV için GAT kodunu uygula
        for i, gat_kodu in enumerate(tahminler):
            # GAT kodunu ROV'a kaydet
            app.rovs[i].gat_kodu = gat_kodu
            
            # Lider her zaman kırmızı, diğerleri GAT koduna göre renklenir
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            # Label güncelle
            app.rovs[i].label.scale = 6000
            app.rovs[i].label.y = 300
            app.rovs[i].label.color = app.rovs[i].color
            app.rovs[i].label.background = False
            
            # GAT durumunu göster
            durum_metni = durum_txts[gat_kodu] if 0 <= gat_kodu < len(durum_txts) else f"GAT:{gat_kodu}"
            ai_durum = "" if ai_aktif else "\n[AI OFF]"
            app.rovs[i].label.text = f"{durum_metni}{i}{ai_durum}"
        
    except Exception as e:
        print(f"❌ [HATA] Update döngüsü: {e}")
        import traceback
        traceback.print_exc()

app.set_update_function(update)

# 3. ÇALIŞTIRMA
if __name__ == "__main__":
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        print("\n🛑 Simülasyon durduruldu.")
    finally: 
        os.system('stty sane')
        os._exit(0)
