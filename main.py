from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo, TemelGNC
from FiratROVNet.config import cfg
from ursina import *  # type: ignore[reportMissingImports]
from ursina import time as utime, mouse, Vec3, time # type: ignore[reportMissingImports]  # FPS icin dt; mouse: girdi
import numpy as np
import os
import time
from datetime import datetime

# ==========================================
# 1. KURULUM VE YAPILANDIRMA
# ==========================================
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()

# Simülasyonu oluştur: 6 ROV, 6 Ada, 200m havuz yarıçapı
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

print("✅ Sistem aktif. Minimap: sol tıkla. Ekran görüntüsü (makale kalitesi): F tuşu → Pictures/")


# ==========================================
# 2. ANA DÖNGÜ (UPDATE)
# ==========================================
# FPS sabitleme: hedef FPS (tum kareler ~esit sure); 0 = limit yok
# FPS gosterimi: son N karenin ortalamasi (titreme azalir)
_fps_history = []
_FPS_HISTORY_SIZE = 10

def update():
    """Ana simülasyon döngüsü. Frame süresi hedef FPS ile eşitlenir."""
    # Verileri çek (filo.get metodunuza göre uyarlandı)
    gps = filo.get(bilgi_rov_id, "gps") or Vec3(0,0,0)
    batarya = filo.get(bilgi_rov_id, "batarya") or 0
            
    # Hız verileri (Vektör büyüklükleri)
    l_vel_vec = filo.rovs[bilgi_rov_id].velocity or Vec3(0,0,0)
    a_vel_vec = filo.rovs[bilgi_rov_id].rotation_y or 0
            
    l_speed = l_vel_vec.length()
    a_speed = a_vel_vec

    # Metni oluştur
    grup_id = filo.rovs[bilgi_rov_id].group_id
    rol = filo.rovs[bilgi_rov_id].get("rol")
    metin = ""
    if rol == 1:
        metin = f"Lider-{bilgi_rov_id} "
    else:
        metin = f"Takipci-{bilgi_rov_id} "

    metin += f"Grup-{grup_id}"
    avg_fps = 0

    dt = getattr(utime, 'dt', 0) or 0.016

    instant_fps = (1.0 / dt) if dt > 0 else 0

    _fps_history.append(instant_fps)

    if len(_fps_history) > _FPS_HISTORY_SIZE:
        _fps_history.pop(0)
    avg_fps = int(sum(_fps_history) / len(_fps_history)) if _fps_history else 0



    # Verileri her karede güncellerken bu formatı kullanın:
    info_text = (
        f"<yellow>       FPS: {avg_fps}<default>\n" +   # FPS Sarı
        f"<orange>{metin}<default>\n" +                # Başlık (Lider) Turuncu
        f"<cyan>GPS: {int(gps[0])}, {int(gps[1])}, {int(gps[2])}<default>\n" + # GPS Turkuaz
        f"<azure>BAT: {batarya:.2f} J<default>\n" +       # Batarya Azure mavisi
        f"<lime>VEL: {l_speed:.2f} m/s<default>\n" +      # Hız Lime yeşili
        f"<gold>ANG: {a_speed:.2f} rad/s<default>"        # Açısal hız Altın sarısı
    )

    app.rov_label.text = info_text



    tahminler = np.zeros(len(app.rovs), dtype=int)
    filo.guncelle_gat_analizi(tahminler)
    filo.guncelle_hepsi(tahminler, guncelle_gorseller=True)


app.set_update_function(update)





# ==========================================
# 3. GİRDİ YÖNETİMİ (MOUSE)
# ==========================================

bilgi_rov_id = 0

def input(key):
    """Mouse ve keyboard girdilerini işle."""
    global bilgi_rov_id

    if key in ('f', 'F'):
        # Ekran goruntusu: Pictures/ sim_capture_YYYYMMDD_HHMMSS.png (yuksek kalite, makale icin uygun)
        try:
            from ursina import application
            from panda3d.core import Filename
            _script_dir = os.path.dirname(os.path.abspath(__file__))
            _pictures = os.path.join(_script_dir, "Pictures")
            os.makedirs(_pictures, exist_ok=True)
            _ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            _name = f"sim_capture_{_ts}.png"
            _path = os.path.join(_pictures, _name)
            _path_abs = os.path.abspath(_path)
            base = getattr(application, "base", None)
            if base is not None and hasattr(base, "win"):
                if base.win.saveScreenshot(Filename.fromOsSpecific(_path_abs)):
                    print(f"📸 Ekran goruntusu kaydedildi: {_path}")
                else:
                    print("📸 Ekran goruntusu kaydedilemedi.")
            else:
                print("📸 Pencere (base.win) bulunamadi.")
        except Exception as e:
            print(f"📸 Ekran goruntusu hatasi: {e}")

    if key == 'p':
        lider_id, _ = filo.find_leader_info(g_id=filo.rovs[bilgi_rov_id].group_id)
        lider_rov = filo.find_rov_by_id(lider_id) if lider_id is not None else None
        if lider_rov:
            filo.entity_patlat(lider_rov)

    if key == "r":
        bilgi_rov_id += 1
        bilgi_rov_id %= len(filo.rovs)
        filo.kamera_ayarla(rov_id=bilgi_rov_id)
        print(f"🔄 Aktif ROV: {bilgi_rov_id}")

    
    if key == 'left mouse down':
        # Eğer tıklanan nesne minimap ise
        if hasattr(app, 'minimap') and mouse.hovered_entity == app.minimap:
            # Tıklanan yerin koordinatını havuz boyutuna göre çevir
            local_pos = mouse.point 
            havuz_tam_cap = 400 
            sim_x = local_pos.x * havuz_tam_cap
            sim_y = local_pos.y * havuz_tam_cap
            
            # Mevcut derinliği grubun liderinden al
            lider_id, lider_gps = filo.find_leader_info(g_id=filo.rovs[bilgi_rov_id].group_id)
            mevcut_z = -5 #lider_gps[2] if lider_gps else -10
            
            # Benzersiz ID oluştur ve hedefi kaydet
            filo.target_counter += 1
            new_id = filo.target_counter
            new_target_pos = (sim_x, sim_y, mevcut_z)

            # Kuyruğa paket olarak ekle (grup bazli)
            filo.nav_queue.setdefault(filo.rovs[bilgi_rov_id].group_id, []).append({'pos': new_target_pos, 'id': new_id})
            filo.current_target_id.setdefault(filo.rovs[bilgi_rov_id].group_id, None)
            
            # Görseli oluştur
            filo._hedef_gorsel_olustur(sim_x, sim_y, mevcut_z, id=new_id, debug=False)
            
            bekleyen = len(filo.nav_queue.get(filo.rovs[bilgi_rov_id].group_id, []))
            print(f"📥 [KUYRUK] Grup-{filo.rovs[bilgi_rov_id].group_id} hedef {new_id} eklendi | Bekleyen: {bekleyen}")

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