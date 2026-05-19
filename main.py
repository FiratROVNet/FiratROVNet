import os

import numpy as np
from ursina import Vec3, time as utime

from FiratROVNet.config import PerformansAyarlari
from FiratROVNet.gnc import Filo
from FiratROVNet.kutuphane.moduls.profiler import Profiler
from FiratROVNet.main_runtime import (
    FrameScheduler,
    KomutaArayuzu,
    RerunRecorder,
    akademik_gorsel_kaydet,
    aktif_grup_idleri,
    aktif_rov_idleri,
    kisayol_paneli_olustur,
    konsol_komutlari_ekle,
    lider_patlat,
    minimap_tiklama_isle,
    ui_minimap_secim_iptal,
    sac_hud_toggle,
    sonraki_grup,
    sonraki_rov,
    sonraki_sac_rov,
    tahminler_al,
    uygulamayi_calistir,
)
from FiratROVNet.kutuphane.moduls.Panels import MotorHUD, SACEgitimHUD
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.tools.rerun_ayarla import rerun_sahne_logla


script_dir = os.path.dirname(os.path.abspath(__file__))

print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
app._firat_script_dir = script_dir
#app.sim_olustur(n_rovs=(6, 4), n_islands=4, havuz_genisligi=200, rov_model="submarine")
app.sim_olustur(n_islands=4, havuz_genisligi=200, rov_model="submarine",seed=1)

filo = Filo(ortam_ref=app)
motor_hud = filo.panels.register("motor_hud", MotorHUD(filo))
sac_hud = filo.panels.register("sac_hud", SACEgitimHUD(filo))
filo._sac_hud_visible = sac_hud.visible

if getattr(app, "rov_label", None) is not None and app.rov_label.background is not None:
    app.rov_label.background.scale_y = 2.2

kisayol_paneli_olustur()
konsol_komutlari_ekle(app, filo)
rerun = RerunRecorder(script_dir)
ui = KomutaArayuzu(script_dir, app, filo)
app.konsola_ekle("ui_ac", ui.open)
app.konsola_ekle("ui_kapat", ui.close)
app.konsola_ekle("ui_rov_ekle", ui.rov_ekle)

print(
    "✅ Sistem aktif. Minimap sol tık = hedef (A*) | "
    "'Haritadan Seç' = çokgen alan (A* kapalı) | Esc = iptal"
)

fps_history = []
FPS_HISTORY_SIZE = 10
rerun_step = 0
tahminler_cache = np.zeros(0, dtype=int)
last_hud_text = ""
bilgi_rov_id = 0
aktif_grup_index = 0
scheduler = FrameScheduler()


def gps_al(rov_id):
    gps = filo.get(rov_id, "gps")
    return gps if gps is not None else Vec3(0, 0, 0)


def aktif_rov_secimini_dogrula():
    global bilgi_rov_id, aktif_grup_index

    if filo.find_rov_by_id(bilgi_rov_id) is not None:
        return
    ids = aktif_rov_idleri(filo)
    if not ids:
        return
    bilgi_rov_id = ids[0]
    mevcut_grup = getattr(filo.find_rov_by_id(bilgi_rov_id), "group_id", None)
    grup_idleri = aktif_grup_idleri(filo)
    if mevcut_grup in grup_idleri:
        aktif_grup_index = grup_idleri.index(mevcut_grup)
    if not filo.camera_manager.aktif_kamera_listesi():
        filo.kamera_ayarla(rov_id=bilgi_rov_id)


def hud_metni_olustur():
    rov = filo.find_rov_by_id(bilgi_rov_id)
    if rov is None:
        avg_fps = int(sum(fps_history) / len(fps_history)) if fps_history else 0
        return (
            f"<yellow>       FPS: {avg_fps}<default>\n"
            f"<orange>ROV yok<default>\n"
            f"<cyan>Runtime konsoldan ROV eklenebilir<default>\n"
            f"<azure>BAT: --<default>\n"
            f"<lime>VEL: --<default>\n"
            f"<gold>ANG: --<default>"
        )
    gps = gps_al(bilgi_rov_id)
    batarya = filo.get(bilgi_rov_id, "batarya") or 0
    velocity = getattr(rov, "velocity", Vec3(0, 0, 0)) or Vec3(0, 0, 0)
    angular_speed = getattr(rov, "rotation_y", 0) or 0
    grup_id = getattr(rov, "group_id", 0)
    rol = rov.get("rol") if rov else 0
    rol_metin = f"Lider-{bilgi_rov_id} " if rol == 1 else f"Takipci-{bilgi_rov_id} "
    avg_fps = int(sum(fps_history) / len(fps_history)) if fps_history else 0
    return (
        f"<yellow>       FPS: {avg_fps}<default>\n"
        f"<orange>{rol_metin}Grup-{grup_id}<default>\n"
        f"<cyan>GPS: {int(gps[0])}, {int(gps[1])}, {int(gps[2])}<default>\n"
        f"<azure>BAT: {batarya:.2f} J<default>\n"
        f"<lime>VEL: {velocity.length():.2f} m/s<default>\n"
        f"<gold>ANG: {angular_speed:.2f} rad/s<default>"
    )


def update():
    """Ana simülasyon döngüsü. Frame süresi hedef FPS ile eşitlenir."""
    global rerun_step, tahminler_cache, last_hud_text

    dt = getattr(utime, "dt", 0) or 0.016
    instant_fps = (1.0 / dt) if dt > 0 else 0
    fps_history.append(instant_fps)
    if len(fps_history) > FPS_HISTORY_SIZE:
        fps_history.pop(0)

    aktif_rov_secimini_dogrula()

    if scheduler.due("hud", PerformansAyarlari.HUD_HZ, dt):
        Profiler.start("0_hud_text_update")
        info_text = hud_metni_olustur()
        if info_text != last_hud_text:
            app.rov_label.text = info_text
            last_hud_text = info_text
        Profiler.end("0_hud_text_update")

    if scheduler.due("motor_hud", PerformansAyarlari.MOTOR_HUD_HZ, dt):
        Profiler.start("0_motor_hud_update")
        motor_hud.update(bilgi_rov_id)
        Profiler.end("0_motor_hud_update")

    if sac_hud.visible and scheduler.due("sac_train", 10.0, dt):
        mevcut_rov = filo.find_rov_by_id(bilgi_rov_id)
        varsayilan_grup_id = getattr(mevcut_rov, "group_id", None)
        egitim_rovleri = filo.sac.canli_egitim_rovleri_al(varsayilan_grup_id=varsayilan_grup_id)
        if egitim_rovleri:
            Profiler.start("0_sac_canli_egitim")
            filo.sac.canli_egitim_adimi(rov_id=egitim_rovleri)
            Profiler.end("0_sac_canli_egitim")

    if scheduler.due("sac_hud", 10.0, dt):
        Profiler.start("0_sac_hud_update")
        filo._sac_hud_visible = sac_hud.visible
        sac_hud.update()
        Profiler.end("0_sac_hud_update")

    tahminler_cache = tahminler_al(app, tahminler_cache)
    if scheduler.due("gat", PerformansAyarlari.GAT_HZ, dt):
        Profiler.start("0_guncelle_gat_analizi")
        tahminler_cache.fill(0)
        filo.guncelle_gat_analizi(tahminler_cache)
        Profiler.end("0_guncelle_gat_analizi")

    guncelle_gorseller = scheduler.due("gorseller", PerformansAyarlari.GORSELLER_HZ, dt)
    guncelle_lider = scheduler.due("lider", PerformansAyarlari.LIDER_HZ, dt)
    filo.guncelle_hepsi(tahminler_cache, guncelle_gorseller=guncelle_gorseller, guncelle_lider=guncelle_lider)

    if not rerun.sink_busy and scheduler.due("rerun", PerformansAyarlari.RERUN_HZ, dt):
        Profiler.start("0_rerun_sahne_logla")
        rerun_sahne_logla(app=app, filo=filo, step=rerun_step)
        Profiler.end("0_rerun_sahne_logla")
    rerun_step += 1

    ui.guncelle(scheduler, dt)


app.set_update_function(update)


def input(key):
    """Mouse ve keyboard girdilerini işle."""
    global bilgi_rov_id, aktif_grup_index

    if key in ("f", "F"):
        akademik_gorsel_kaydet(script_dir)
    if key in ("m", "M"):
        motor_hud.toggle()
    if key in ("b", "B"):
        filo.toggle_pid_ui()
    if key in ("v", "V"):
        rerun.toggle()
    if key in ("u", "U"):
        ui.toggle()
    if key in ("e", "E"):
        sac_hud_toggle(filo, sac_hud, bilgi_rov_id)
    if key in ("2", "num 2") and sac_hud.visible:
        sonraki_sac_rov(filo, sac_hud)
    if key in ("g", "G"):
        bilgi_rov_id, aktif_grup_index = sonraki_grup(filo, bilgi_rov_id, aktif_grup_index)
    if key == "p":
        lider_patlat(filo, bilgi_rov_id)
    if key == "r":
        bilgi_rov_id = sonraki_rov(filo, bilgi_rov_id)
        if not filo.rovs:
            return
        mevcut_grup = getattr(filo.find_rov_by_id(bilgi_rov_id), "group_id", None)
        grup_idleri = aktif_grup_idleri(filo)
        if mevcut_grup in grup_idleri:
            aktif_grup_index = grup_idleri.index(mevcut_grup)
    if key == "escape":
        if getattr(app, "_ui_minimap_picker", None):
            ui_minimap_secim_iptal(app)
    if key == "left mouse down":
        minimap_tiklama_isle(app, filo, bilgi_rov_id)


if __name__ == "__main__":
    uygulamayi_calistir(app, rerun)
