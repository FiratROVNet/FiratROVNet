from FiratROVNet.kutuphane.moduls.profiler import Profiler
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo, TemelGNC
from FiratROVNet.config import cfg
from FiratROVNet.config import PerformansAyarlari
from FiratROVNet.motor_hud import MotorHUD
from ursina import *  # type: ignore[reportMissingImports]
from ursina import time as utime, mouse, Vec3, time # type: ignore[reportMissingImports]  # FPS icin dt; mouse: girdi
import numpy as np
import os
import time
import threading
from rerun_ayarla import QR, rerun_baslat, rerun_sahne_logla, rerun_kayit_baslat, rerun_kayit_durdur

# ==========================================
# 1. KURULUM VE YAPILANDIRMA
# ==========================================
print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()

# Simülasyonu oluştur: 6 ROV, 6 Ada, 200m havuz yarıçapı
app.sim_olustur(n_rovs=(6,4), n_islands=4, havuz_genisligi=200, rov_model='submarine')

# Filo sistemini ortamla birlikte oluştur (otomatik bağlantı)
# GAT modeli ve navigasyon kuyruğu da Filo içinde initialize edilir
filo = Filo(ortam_ref=app)
motor_hud = MotorHUD(filo)
if getattr(app, "rov_label", None) is not None and app.rov_label.background is not None:
    app.rov_label.background.scale_y = 2.2

shortcut_root = Entity(parent=camera.ui, position=(0.8, 0.28, -9))
shortcut_bg = Entity(
    parent=shortcut_root,
    model="quad",
    scale=(0.18, 0.08),
    color=color.black,
    z=0.04,
)
shortcut_bg.alpha = 0.48
_shortcut_border_color = color.azure
for _shortcut_border in (
    Entity(parent=shortcut_root, model="quad", position=(0, 0.040, -0.04), scale=(0.18, 0.002), color=_shortcut_border_color),
    Entity(parent=shortcut_root, model="quad", position=(0, -0.040, -0.04), scale=(0.18, 0.002), color=_shortcut_border_color),
    Entity(parent=shortcut_root, model="quad", position=(-0.090, 0, -0.04), scale=(0.002, 0.08), color=_shortcut_border_color),
    Entity(parent=shortcut_root, model="quad", position=(0.090, 0, -0.04), scale=(0.002, 0.08), color=_shortcut_border_color),
):
    _shortcut_border.alpha = 0.30
shortcut_text = Text(
    parent=shortcut_root,
    text="<white>M<default> Motor   <white>B<default> PID\n<white>R<default> ROV     <white>G<default> Grup\n<white>F<default> Görsel  <white>V<default> REC",
    position=(0, 0, 0),
    origin=(0, 0),
    scale=0.7,
    color=color.gray,
)

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

from datetime import datetime

_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
_script_dir = os.path.dirname(os.path.abspath(__file__))
_kayit_klasoru = os.path.join(_script_dir, "Videos", "Rerun")
os.makedirs(_kayit_klasoru, exist_ok=True)
_dosya_yolu = None
_rerun_recording = False
_rerun_sink_busy = False
_aktif_grup_index = 0

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
_tahminler_cache = np.zeros(0, dtype=int)
_last_hud_text = ""


class FrameScheduler:
    def __init__(self):
        self._accum = {}

    def due(self, name, hz, dt, first=True):
        if hz is None or hz <= 0:
            return False
        interval = 1.0 / float(hz)
        current = self._accum.get(name)
        if current is None:
            self._accum[name] = 0.0
            return bool(first)
        current += dt
        if current >= interval:
            self._accum[name] = current % interval
            return True
        self._accum[name] = current
        return False


_scheduler = FrameScheduler()


def _tahminler_al():
    global _tahminler_cache
    rov_sayisi = len(app.rovs)
    if _tahminler_cache.shape[0] != rov_sayisi:
        _tahminler_cache = np.zeros(rov_sayisi, dtype=int)
    return _tahminler_cache


def _gps_al(rov_id):
    gps = filo.get(rov_id, "gps")
    return gps if gps is not None else Vec3(0, 0, 0)

def _aktif_grup_idleri():
    gruplar = []
    try:
        for g_id, grup in filo.g_rovs.items():
            aktif = [r for r in (grup or []) if r and not getattr(r, "is_destroyed", False)]
            if aktif:
                gruplar.append(g_id)
    except Exception:
        pass
    return sorted(gruplar)


def _gruptaki_ilk_rov_id(g_id):
    try:
        grup = filo.g_rovs.get(g_id, [])
        aktif = [r for r in (grup or []) if r and not getattr(r, "is_destroyed", False)]
        aktif.sort(key=lambda r: getattr(r, "id", 0))
        if aktif:
            return getattr(aktif[0], "id", None)
    except Exception:
        return None
    return None


def _rerun_kayit_dosyasi_olustur():
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(_kayit_klasoru, f"simulasyon_{ts}.rrd")


def _rerun_kayit_toggle():
    global _rerun_recording, _dosya_yolu, _rerun_sink_busy
    if _rerun_sink_busy:
        print("⏳ Rerun kayıt durumu değiştiriliyor, lütfen bekle...")
        return

    _rerun_sink_busy = True
    if not _rerun_recording:
        _dosya_yolu = _rerun_kayit_dosyasi_olustur()
        print(f"🎥 Rerun kaydı başlatılıyor: {_dosya_yolu}")
        threading.Thread(target=_rerun_kayit_baslat_async, args=(_dosya_yolu,), daemon=True).start()
        return

    print(f"⏹️ Rerun kaydı durduruluyor: {_dosya_yolu}")
    threading.Thread(target=_rerun_kayit_durdur_async, daemon=True).start()


def _rerun_kayit_baslat_async(dosya_yolu):
    global _rerun_recording, _rerun_sink_busy
    try:
        if rerun_kayit_baslat(_rr_runtime, dosya_yolu):
            _rerun_recording = True
            print(f"✅ Rerun kaydı başladı: {dosya_yolu}")
    finally:
        _rerun_sink_busy = False


def _rerun_kayit_durdur_async():
    global _rerun_recording, _rerun_sink_busy
    try:
        if rerun_kayit_durdur(_rr_runtime):
            print(f"✅ Rerun kaydı durdu: {_dosya_yolu}")
            _rerun_recording = False
    finally:
        _rerun_sink_busy = False

def update():
    """Ana simülasyon döngüsü. Frame süresi hedef FPS ile eşitlenir."""
    global _rerun_step, _last_hud_text
    dt = getattr(utime, 'dt', 0) or 0.016
    instant_fps = (1.0 / dt) if dt > 0 else 0
    _fps_history.append(instant_fps)
    if len(_fps_history) > _FPS_HISTORY_SIZE:
        _fps_history.pop(0)

    if _scheduler.due("hud", PerformansAyarlari.HUD_HZ, dt):
        Profiler.start("0_hud_text_update")
        rov = filo.rovs[bilgi_rov_id] if 0 <= bilgi_rov_id < len(filo.rovs) else None
        gps = _gps_al(bilgi_rov_id)
        batarya = filo.get(bilgi_rov_id, "batarya") or 0
        l_vel_vec = getattr(rov, "velocity", Vec3(0,0,0)) or Vec3(0,0,0)
        a_speed = getattr(rov, "rotation_y", 0) or 0
        grup_id = getattr(rov, "group_id", 0)
        rol = rov.get("rol") if rov else 0
        rol_metin = f"Lider-{bilgi_rov_id} " if rol == 1 else f"Takipci-{bilgi_rov_id} "
        avg_fps = int(sum(_fps_history) / len(_fps_history)) if _fps_history else 0
        info_text = (
            f"<yellow>       FPS: {avg_fps}<default>\n"
            f"<orange>{rol_metin}Grup-{grup_id}<default>\n"
            f"<cyan>GPS: {int(gps[0])}, {int(gps[1])}, {int(gps[2])}<default>\n"
            f"<azure>BAT: {batarya:.2f} J<default>\n"
            f"<lime>VEL: {l_vel_vec.length():.2f} m/s<default>\n"
            f"<gold>ANG: {a_speed:.2f} rad/s<default>"
        )
        if info_text != _last_hud_text:
            app.rov_label.text = info_text
            _last_hud_text = info_text
        Profiler.end("0_hud_text_update")

    if _scheduler.due("motor_hud", PerformansAyarlari.MOTOR_HUD_HZ, dt):
        Profiler.start("0_motor_hud_update")
        motor_hud.update(bilgi_rov_id)
        Profiler.end("0_motor_hud_update")

    tahminler = _tahminler_al()
    if _scheduler.due("gat", PerformansAyarlari.GAT_HZ, dt):
        Profiler.start("0_guncelle_gat_analizi")
        tahminler.fill(0)
        filo.guncelle_gat_analizi(tahminler)
        Profiler.end("0_guncelle_gat_analizi")

    guncelle_gorseller = _scheduler.due("gorseller", PerformansAyarlari.GORSELLER_HZ, dt)
    guncelle_lider = _scheduler.due("lider", PerformansAyarlari.LIDER_HZ, dt)
    filo.guncelle_hepsi(tahminler, guncelle_gorseller=guncelle_gorseller, guncelle_lider=guncelle_lider)

    if not _rerun_sink_busy and _scheduler.due("rerun", PerformansAyarlari.RERUN_HZ, dt):
        Profiler.start("0_rerun_sahne_logla")
        rerun_sahne_logla(app=app, filo=filo, step=_rerun_step)
        Profiler.end("0_rerun_sahne_logla")
    _rerun_step += 1


app.set_update_function(update)





# ==========================================
# 3. GİRDİ YÖNETİMİ (MOUSE)
# ==========================================

bilgi_rov_id = 0

def input(key):
    """Mouse ve keyboard girdilerini işle."""
    global bilgi_rov_id, _aktif_grup_index

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

    if key in ('m', 'M'):
        motor_hud.toggle()

    if key in ('b', 'B'):
        filo.toggle_pid_ui()

    if key in ('v', 'V'):
        _rerun_kayit_toggle()

    if key in ('g', 'G'):
        grup_idleri = _aktif_grup_idleri()
        if grup_idleri:
            mevcut_grup = getattr(filo.rovs[bilgi_rov_id], "group_id", None)
            if mevcut_grup in grup_idleri:
                _aktif_grup_index = (grup_idleri.index(mevcut_grup) + 1) % len(grup_idleri)
            else:
                _aktif_grup_index = (_aktif_grup_index + 1) % len(grup_idleri)
            hedef_grup = grup_idleri[_aktif_grup_index]
            ilk_rov_id = _gruptaki_ilk_rov_id(hedef_grup)
            if ilk_rov_id is not None:
                bilgi_rov_id = int(ilk_rov_id)
                filo.kamera_ayarla(rov_id=bilgi_rov_id)
                print(f"🔄 Aktif Grup: {hedef_grup} | İzlenen ROV: {bilgi_rov_id}")

    if key == 'p':
        lider_bilgi = filo.find_leader_info(g_id=filo.rovs[bilgi_rov_id].group_id)
        lider_id = lider_bilgi[0] if lider_bilgi else None
        lider_rov = filo.find_rov_by_id(lider_id) if lider_id is not None else None
        if lider_rov:
            filo.entity_patlat(lider_rov)

    if key == "r":
        bilgi_rov_id += 1
        bilgi_rov_id %= len(filo.rovs)
        filo.kamera_ayarla(rov_id=bilgi_rov_id)
        mevcut_grup = getattr(filo.rovs[bilgi_rov_id], "group_id", None)
        grup_idleri = _aktif_grup_idleri()
        if mevcut_grup in grup_idleri:
            _aktif_grup_index = grup_idleri.index(mevcut_grup)
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
            
            group_id = filo.rovs[bilgi_rov_id].group_id
            lider_bilgi = filo.find_leader_info(g_id=group_id)
            lider_id = lider_bilgi[0] if lider_bilgi else None
            lider_gps = lider_bilgi[1] if lider_bilgi else None
            if lider_id is None:
                print(f"⚠️ [NAV] Grup-{group_id} icin aktif lider bulunamadi.")
                return

            mevcut_z = -20 #lider_gps[2] if lider_gps else -10.0
            
            # Benzersiz ID oluştur ve hedefi kaydet
            filo.target_counter += 1
            new_id = filo.target_counter
            new_target_pos = (sim_x, sim_y, mevcut_z)
            aktif_rota = bool(filo._git_nokta_listesi.get(lider_id))
            aktif_hedef = filo.hedef(rov_id=lider_id)

            # Görseli oluştur
            filo._hedef_gorsel_olustur(sim_x, sim_y, mevcut_z, id=new_id, debug=False)

            # Grup bosta ise hedefi dogrudan liderde baslat; mesgulse kuyruga ekle.
            if not aktif_rota and aktif_hedef is None:
                filo.current_target_id[group_id] = new_id
                print(f"🚀 [NAV] Grup-{group_id} hedef {new_id} dogrudan baslatiliyor")
                filo.git_path(lider_id, new_target_pos, isaret=True)
            else:
                filo.nav_queue.setdefault(group_id, []).append({'pos': new_target_pos, 'id': new_id})
                filo.current_target_id.setdefault(group_id, None)
                bekleyen = len(filo.nav_queue.get(group_id, []))
                print(f"📥 [KUYRUK] Grup-{group_id} hedef {new_id} eklendi | Bekleyen: {bekleyen}")

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
        if _rerun_recording:
            try:
                rerun_kayit_durdur(_rr_runtime)
            except Exception:
                pass
        try:
            Profiler.rapor_ver()
        except Exception:
            pass
        os.system('stty sane')
        os._exit(0)
