from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo, TemelGNC
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import os

# ==========================================
# 1. KURULUM VE YAPILANDIRMA
# ==========================================
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
# Simülasyonu oluştur (havuz boyutu FiratROVNet/config.py HavuzAyarlari'dan alınır)
app.sim_olustur(n_rovs=(4,3,), n_islands=4, rov_model='submarine')

# Filo sistemini ortamla birlikte oluştur (otomatik bağlantı)
# GAT modeli ve navigasyon kuyruğu da Filo içinde initialize edilir
filo = Filo(ortam_ref=app)

# Konsol fonksiyonları
app.konsola_ekle("git", lambda rov_id, x, z, y=None, ai=True: filo.git(rov_id, x, z, y, ai))
app.konsola_ekle("move", lambda rov_id, yon, guc=1.0: filo.move(rov_id, yon, guc))
app.konsola_ekle("get", lambda rov_id, veri_tipi: filo.get(rov_id, veri_tipi))
app.konsola_ekle("set", lambda rov_id, ayar_adi, deger: filo.set(rov_id, ayar_adi, deger))
app.konsola_ekle("Ada", lambda ada_id, x=None, y=None: app.Ada(ada_id, x, y))
app.konsola_ekle("ROV", lambda rov_id, x=None, y=None, z=None: app.ROV(rov_id, x, y, z))
app.konsola_ekle("filo", filo)
app.konsola_ekle("rovs", app.rovs)
app.konsola_ekle("cfg", cfg)
app.konsola_ekle("nav_queue", filo.nav_queue)  # Kuyruğu konsoldan izleyebilirsin

print("✅ Sistem aktif. Minimap üzerinden hedef eklemek için sol tıkla.")


# ==========================================
# 2. ANA DÖNGÜ (UPDATE)
# ==========================================
def update():
    """Ana simülasyon döngüsü (maksimal sadeleştirme)."""
    try:
        # Tahminler array'ı
        tahminler = np.zeros(len(app.rovs), dtype=int)
        
        # GAT analizi
        filo.guncelle_gat_analizi(tahminler)
        
        # Tüm sistem güncellemeleri (guncelle_hepsi içinde):
        # - Navigasyon kuyruğu
        # - Lider yönetimi & path transfer
        # - ROV hasar/GNC işlemleri
        # - Sonar, minimap, engel bulut
        filo.guncelle_hepsi(tahminler)
        
        # Görselleştirme
        
        
    except Exception as e:
        print(f"❌ [HATA] Update döngüsü: {e}")

app.set_update_function(update)





# ==========================================
# 3. GİRDİ YÖNETİMİ (MOUSE)
# ==========================================

grup_id = 0
lider_id = 0

def input(key):
    """Mouse ve keyboard girdilerini işle."""
    global grup_id, lider_id

    if key == 'p':
        lider_id, _ = filo.find_leader_info(g_id=grup_id)
        lider_rov = filo.find_rov_by_id(lider_id) if lider_id is not None else None
        if lider_rov:
            filo.entity_patlat(lider_rov)
    
    if key == 'left mouse down':
        # Eğer tıklanan nesne minimap ise
        if hasattr(app, 'minimap') and mouse.hovered_entity == app.minimap:
            # Tıklanan yerin koordinatını havuz boyutuna göre çevir (config'den)
            local_pos = mouse.point
            havuz_tam_cap = app.havuz_genisligi * 2
            sim_x = local_pos.x * havuz_tam_cap
            sim_y = local_pos.y * havuz_tam_cap
            
            # Mevcut derinliği grubun liderinden al
            lider_id, lider_gps = filo.find_leader_info(g_id=grup_id)
            mevcut_z = -20 #lider_gps[2] if lider_gps else -10
            
            # Benzersiz ID oluştur ve hedefi kaydet
            filo.target_counter += 1
            new_id = filo.target_counter
            new_target_pos = (sim_x, sim_y, mevcut_z)

            # Kuyruğa paket olarak ekle (grup bazli)
            filo.nav_queue.setdefault(grup_id, []).append({'pos': new_target_pos, 'id': new_id})
            filo.current_target_id.setdefault(grup_id, None)
            
            # Görseli oluştur
            filo._hedef_gorsel_olustur(sim_x, sim_y, mevcut_z, id=new_id, debug=False)
            
            bekleyen = len(filo.nav_queue.get(grup_id, []))
            print(f"📥 [KUYRUK] Grup-{grup_id} hedef {new_id} eklendi | Bekleyen: {bekleyen}")

    if key == 'g':
        grup_id += 1
        grup_id %= len(filo.g_rovs)
        print(f"🔄 Aktif Grup: {grup_id}")

# ==========================================
# 4. ÇALIŞTIRMA
# ==========================================
if __name__ == "__main__":
    # Minimap'e tıklanabilmesi için collider ekle
    if hasattr(app, 'minimap') and app.minimap:
        app.minimap.collider = 'box' 
        
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        print("\n🛑 Simülasyon durduruldu.")
    finally: 
        os.system('stty sane')
        os._exit(0)