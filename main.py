from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo, TemelGNC
from FiratROVNet.config import cfg
from ursina import *  # type: ignore[reportMissingImports]
from ursina import time as utime, mouse, Vec3, time # type: ignore[reportMissingImports]  # FPS icin dt; mouse: girdi
import numpy as np
import os
import time
from rerun_ayarla import QR, rerun_baslat, rerun_sahne_logla

# ==========================================
# 1. KURULUM VE YAPILANDIRMA
# ==========================================
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()

# Simülasyonu oluştur: 6 ROV, 6 Ada, 200m havuz yarıçapı
app.sim_olustur(n_rovs=(4,3), n_islands=4, havuz_genisligi=200, rov_model='submarine')

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

_rr_runtime = rerun_baslat(ip_adresi=os.getenv("RR_IP_ADRESI"))

QR(
    ip_adresi=_rr_runtime.get("lan_ip", "127.0.0.1"),
    link=_rr_runtime.get("web_lan_url", ""),
)




print("✅ Sistem aktif. Minimap: sol tıkla. Ekran görüntüsü (makale kalitesi): F tuşu → Pictures/")


# ==========================================
# 2. ANA DÖNGÜ (UPDATE)
# ==========================================
# FPS sabitleme: hedef FPS (tum kareler ~esit sure); 0 = limit yok
# FPS gosterimi: son N karenin ortalamasi (titreme azalir)
_fps_history = []
_FPS_HISTORY_SIZE = 10
_rerun_step = 0

def update():
    """Ana simülasyon döngüsü. Frame süresi hedef FPS ile eşitlenir."""
    global _rerun_step
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
    rerun_sahne_logla(app=app, filo=filo, step=_rerun_step)
    _rerun_step += 1


app.set_update_function(update)





# ==========================================
# 3. GİRDİ YÖNETİMİ (MOUSE)
# ==========================================

bilgi_rov_id = 0

def input(key):
    """Mouse ve keyboard girdilerini işle."""
    global bilgi_rov_id

    if key in ('f', 'F'):
            try:
                from ursina import application, camera, window, destroy, invoke
                from panda3d.core import Filename
                from PIL import Image
                from datetime import datetime
                import os

                # --- AYARLAR (Akademik Standartlar) ---
                TARGET_WIDTH = 1280  # Makale için ideal genişlik
                TARGET_HEIGHT = 720  # Makale için ideal yükseklik
                UI_GIZLE = True      # Makale resminde minimap/yazı görünmesin
                
                _script_dir = os.path.dirname(os.path.abspath(__file__))
                _pictures = os.path.join(_script_dir, "Pictures")
                os.makedirs(_pictures, exist_ok=True)
                
                _ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                _temp_path = os.path.join(_pictures, f"temp_{_ts}.png")
                _final_name = f"article_capture_{_ts}.png"
                _final_path = os.path.join(_pictures, _final_name)

                # 1. UI elemanlarını geçici olarak gizle (Profesyonel görünüm için)
                if UI_GIZLE:
                    camera.ui.enabled = False

                # 2. Ekran görüntüsünü al (O anki tam çözünürlükte)
                base = getattr(application, "base", None)
                if base and base.win:
                    base.win.saveScreenshot(Filename.fromOsSpecific(_temp_path))
                    
                    # 3. Pillow ile işle (Küçültme ve Kalite Artırma)
                    # Küçük bir bekleme gerekebilir ama saveScreenshot senkrondur
                    with Image.open(_temp_path) as img:
                        # Görüntüyü Lanczos filtresiyle yeniden boyutlandır 
                        # (Bu işlem tırtıklanmayı önler ve keskinliği korur)
                        img_resized = img.resize((TARGET_WIDTH, TARGET_HEIGHT), Image.Resampling.LANCZOS)
                        
                        # DPI ayarı ekleyerek kaydet (300 DPI akademik baskı standardıdır)
                        img_resized.save(_final_path, "PNG", dpi=(300, 300), optimize=True)
                    
                    # Geçici ham dosyayı sil
                    if os.path.exists(_temp_path):
                        os.remove(_temp_path)

                    print(f"✅ Akademik kalite görsel kaydedildi (300 DPI): {_final_name}")
                
                # 4. UI'ı geri aç
                if UI_GIZLE:
                    camera.ui.enabled = True

            except Exception as e:
                print(f"📸 Görsel işleme hatası: {e}")

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
            if local_pos is None:
                return
            havuz_tam_cap = 400 
            sim_x = local_pos.x * havuz_tam_cap
            sim_y = local_pos.y * havuz_tam_cap
            
            # Mevcut derinliği grubun liderinden al
            lider_id, lider_gps = filo.find_leader_info(g_id=filo.rovs[bilgi_rov_id].group_id)
            mevcut_z = -20 #lider_gps[2] if lider_gps else -10
            
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