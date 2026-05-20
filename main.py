import os

from FiratROVNet.gnc import Filo
from FiratROVNet.main_runtime import (
    AnaDonguRuntime,
    KomutaArayuzu,
    RerunRecorder,
    RuntimeOzellikleri,
    kisayol_paneli_olustur,
    konsol_komutlari_ekle,
    uygulamayi_calistir,
)
from FiratROVNet.kutuphane.moduls.Panels import MotorHUD, ProfilerHUD, SACEgitimHUD
from FiratROVNet.simulasyon import Ortam


script_dir = os.path.dirname(os.path.abspath(__file__))

print("🔵 Fırat-GNC Sistemi Başlatılıyor...")
app = Ortam()
app._firat_script_dir = script_dir
#app.sim_olustur(n_rovs=(6, 4), n_islands=4, havuz_genisligi=200, rov_model="submarine")
app.sim_olustur(n_islands=4, havuz_genisligi=200, rov_model="submarine",seed=1)

filo = Filo(ortam_ref=app)
motor_hud = filo.panels.register("motor_hud", MotorHUD(filo, visible=False))
profiler_hud = filo.panels.register("profiler_hud", ProfilerHUD())
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

runtime_ozellikleri = RuntimeOzellikleri(
    hud_text=True,
    motor_hud=True,
    profiler_hud=True,
    sac_train=True,
    sac_hud=True,
    gat=False,          # True yaparsan GAT analizi yeniden aktif olur.
    gorseller=True,
    lider=True,
    rerun=True,
    ui=True,
    pid_ui=True,
    apf_hud=True,
    navigasyon=True,
    rovler=True,
    alan_tarama=True,
)
runtime = AnaDonguRuntime(
    script_dir=script_dir,
    app=app,
    filo=filo,
    motor_hud=motor_hud,
    profiler_hud=profiler_hud,
    sac_hud=sac_hud,
    rerun=rerun,
    ui=ui,
    ozellikler=runtime_ozellikleri,
)
app.konsola_ekle("runtime", runtime)
app.konsola_ekle("ozellikler", runtime_ozellikleri)
app.konsola_ekle("ozellik_ayarla", runtime.ozellik_ayarla)

print(
    "✅ Sistem aktif. Minimap sol tık = hedef (A*) | "
    "'Haritadan Seç' = çokgen alan (A* kapalı) | Esc = iptal"
)

def update():
    runtime.guncelle()


app.set_update_function(update)


def input(key):
    runtime.girdi_isle(key)


if __name__ == "__main__":
    uygulamayi_calistir(app, rerun)
