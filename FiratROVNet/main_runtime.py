import os
import threading
from datetime import datetime

import numpy as np
from ursina import camera, mouse

from FiratROVNet.config import cfg
from FiratROVNet.kutuphane.moduls.Panels import kisayol_paneli_olustur
from FiratROVNet.kutuphane.moduls.profiler import Profiler
from FiratROVNet.tools.rerun_ayarla import (
    QR,
    rerun_baslat,
    rerun_kayit_baslat,
    rerun_kayit_durdur,
)


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


class RerunRecorder:
    def __init__(self, script_dir):
        self.kayit_klasoru = os.path.join(script_dir, "Videos", "Rerun")
        self.dosya_yolu = None
        self.recording = False
        self.sink_busy = False
        os.makedirs(self.kayit_klasoru, exist_ok=True)
        self.runtime = rerun_baslat(ip_adresi=os.getenv("RR_IP_ADRESI"))
        QR(
            ip_adresi=self.runtime.get("lan_ip", "127.0.0.1"),
            link=self.runtime.get("web_lan_url", ""),
        )

    def toggle(self):
        if self.sink_busy:
            print("⏳ Rerun kayıt durumu değiştiriliyor, lütfen bekle...")
            return
        self.sink_busy = True
        if not self.recording:
            self.dosya_yolu = self._kayit_dosyasi_olustur()
            print(f"🎥 Rerun kaydı başlatılıyor: {self.dosya_yolu}")
            threading.Thread(target=self._baslat_async, args=(self.dosya_yolu,), daemon=True).start()
            return
        print(f"⏹️ Rerun kaydı durduruluyor: {self.dosya_yolu}")
        threading.Thread(target=self._durdur_async, daemon=True).start()

    def stop_if_recording(self):
        if not self.recording:
            return
        try:
            rerun_kayit_durdur(self.runtime)
        except Exception:
            pass

    def _kayit_dosyasi_olustur(self):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self.kayit_klasoru, f"simulasyon_{ts}.rrd")

    def _baslat_async(self, dosya_yolu):
        try:
            if rerun_kayit_baslat(self.runtime, dosya_yolu):
                self.recording = True
                print(f"✅ Rerun kaydı başladı: {dosya_yolu}")
        finally:
            self.sink_busy = False

    def _durdur_async(self):
        try:
            if rerun_kayit_durdur(self.runtime):
                print(f"✅ Rerun kaydı durdu: {self.dosya_yolu}")
                self.recording = False
        finally:
            self.sink_busy = False


def konsol_komutlari_ekle(app, filo):
    app.konsola_ekle("git", lambda rov_id, x, z, y=None, ai=True: filo.git(rov_id, x, z, y, ai))
    app.konsola_ekle("move", lambda rov_id, yon, guc=1.0: filo.move(rov_id, yon, guc))
    app.konsola_ekle("get", lambda rov_id, veri_tipi: filo.get(rov_id, veri_tipi))
    app.konsola_ekle("set", lambda rov_id, ayar_adi, deger: filo.set(rov_id, ayar_adi, deger))
    app.konsola_ekle("Ada", lambda ada_id, x=None, y=None: app.Ada(ada_id, x, y))
    app.konsola_ekle("ROV", lambda rov_id, x=None, y=None, z=None: app.ROV(rov_id, x, y, z))
    app.konsola_ekle("filo", filo)
    app.konsola_ekle("panels", getattr(filo, "panels", None))
    app.konsola_ekle("rovs", app.rovs)
    app.konsola_ekle("cfg", cfg)
    app.konsola_ekle("nav_queue", filo.nav_queue)


def tahminler_al(app, tahminler_cache):
    rov_sayisi = len(app.rovs)
    if tahminler_cache.shape[0] != rov_sayisi:
        return np.zeros(rov_sayisi, dtype=int)
    return tahminler_cache


def akademik_gorsel_kaydet(script_dir):
    try:
        from PIL import Image
        from panda3d.core import Filename
        from ursina import application

        pictures = os.path.join(script_dir, "Pictures")
        os.makedirs(pictures, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        temp_path = os.path.join(pictures, f"temp_{ts}.png")
        final_name = f"article_capture_{ts}.png"
        final_path = os.path.join(pictures, final_name)

        camera.ui.enabled = False
        base = getattr(application, "base", None)
        if base and base.win:
            base.win.saveScreenshot(Filename.fromOsSpecific(temp_path))
            with Image.open(temp_path) as img:
                img.resize((1280, 720), Image.Resampling.LANCZOS).save(
                    final_path,
                    "PNG",
                    dpi=(300, 300),
                    optimize=True,
                )
            if os.path.exists(temp_path):
                os.remove(temp_path)
            print(f"✅ Akademik kalite görsel kaydedildi (300 DPI): {final_name}")
        camera.ui.enabled = True
    except Exception as exc:
        camera.ui.enabled = True
        print(f"📸 Görsel işleme hatası: {exc}")


def aktif_grup_idleri(filo):
    gruplar = []
    try:
        for g_id, grup in filo.g_rovs.items():
            aktif = [rov for rov in (grup or []) if rov and not getattr(rov, "is_destroyed", False)]
            if aktif:
                gruplar.append(g_id)
    except Exception:
        pass
    return sorted(gruplar)


def gruptaki_ilk_rov_id(filo, g_id):
    try:
        grup = filo.g_rovs.get(g_id, [])
        aktif = [rov for rov in (grup or []) if rov and not getattr(rov, "is_destroyed", False)]
        aktif.sort(key=lambda rov: getattr(rov, "id", 0))
        if aktif:
            return getattr(aktif[0], "id", None)
    except Exception:
        return None
    return None


def sac_hud_toggle(filo, sac_hud, bilgi_rov_id):
    sac_hud.toggle()
    filo._sac_hud_visible = sac_hud.visible
    if not sac_hud.visible:
        print("🧠 SAC eğitim paneli kapalı")
        return

    mevcut_rov = filo.find_rov_by_id(bilgi_rov_id)
    varsayilan_grup_id = getattr(mevcut_rov, "group_id", None)
    aktif_sac_rov_id = filo.sac.canli_egitim_rov_id_al(varsayilan_grup_id=varsayilan_grup_id)
    if aktif_sac_rov_id is None:
        sac_hud.set_rov_ids([bilgi_rov_id])
        aktif_sac_rov_id = bilgi_rov_id
    sac_hud.set_active_rov_id(aktif_sac_rov_id)
    egitim_rovleri = filo.sac.canli_egitim_rovleri_al(varsayilan_grup_id=varsayilan_grup_id)
    filo.sac.reset(rov_id=egitim_rovleri or aktif_sac_rov_id)
    print(f"🧠 SAC eğitim paneli açık | ROV-{aktif_sac_rov_id}")


def sonraki_sac_rov(filo, sac_hud):
    aktif_sac_rov_id = sac_hud.next_rov()
    if aktif_sac_rov_id is not None:
        filo.sac.reset(rov_id=aktif_sac_rov_id)
        print(f"🧠 SAC grafiği ROV-{aktif_sac_rov_id}")


def sonraki_grup(filo, bilgi_rov_id, aktif_grup_index):
    grup_idleri = aktif_grup_idleri(filo)
    if not grup_idleri:
        return bilgi_rov_id, aktif_grup_index
    mevcut_grup = getattr(filo.rovs[bilgi_rov_id], "group_id", None)
    if mevcut_grup in grup_idleri:
        aktif_grup_index = (grup_idleri.index(mevcut_grup) + 1) % len(grup_idleri)
    else:
        aktif_grup_index = (aktif_grup_index + 1) % len(grup_idleri)
    hedef_grup = grup_idleri[aktif_grup_index]
    ilk_rov_id = gruptaki_ilk_rov_id(filo, hedef_grup)
    if ilk_rov_id is not None:
        bilgi_rov_id = int(ilk_rov_id)
        filo.kamera_ayarla(rov_id=bilgi_rov_id)
        print(f"🔄 Aktif Grup: {hedef_grup} | İzlenen ROV: {bilgi_rov_id}")
    return bilgi_rov_id, aktif_grup_index


def sonraki_rov(filo, bilgi_rov_id):
    bilgi_rov_id += 1
    bilgi_rov_id %= len(filo.rovs)
    filo.kamera_ayarla(rov_id=bilgi_rov_id)
    print(f"🔄 Aktif ROV: {bilgi_rov_id}")
    return bilgi_rov_id


def minimap_hedef_ata(app, filo, bilgi_rov_id):
    if not (hasattr(app, "minimap") and mouse.hovered_entity == app.minimap):
        return
    local_pos = mouse.point
    if local_pos is None:
        return

    sim_x, sim_y, mevcut_z = local_pos.x * 400, local_pos.y * 400, -20
    rov = filo.find_rov_by_id(bilgi_rov_id)
    if rov is None:
        return
    if rov.gnc.mod == 1:
        print(f"⚠️ [NAV] Seçili ROV-{bilgi_rov_id} Takipçi Modunda (mod=1). Hedef ataması reddedildi!")
        return

    aktif_rota = bool(filo._git_nokta_listesi.get(bilgi_rov_id))
    aktif_hedef = filo.hedef(rov_id=bilgi_rov_id)
    filo.target_counter += 1
    new_id = filo.target_counter
    new_target_pos = (sim_x, sim_y, mevcut_z)
    filo._hedef_gorsel_olustur(sim_x, sim_y, mevcut_z, id=new_id, debug=False)

    kuyruk_anahtari = f"rov_{bilgi_rov_id}"
    if not aktif_rota and aktif_hedef is None:
        print(f"🚀 [NAV] ROV-{bilgi_rov_id} (mod=0) hedef {new_id} doğrudan başlatılıyor.")
        filo.current_target_id[kuyruk_anahtari] = new_id
        filo.git_path(bilgi_rov_id, new_target_pos, isaret=True)
        return

    filo.nav_queue.setdefault(kuyruk_anahtari, []).append({"pos": new_target_pos, "id": new_id})
    bekleyen = len(filo.nav_queue.get(kuyruk_anahtari, []))
    print(f"📥 [KUYRUK] ROV-{bilgi_rov_id} hedef {new_id} kendi kuyruğuna eklendi | Bekleyen: {bekleyen}")


def lider_patlat(filo, bilgi_rov_id):
    lider_bilgi = filo.find_leader_info(g_id=filo.rovs[bilgi_rov_id].group_id)
    lider_id = lider_bilgi[0] if lider_bilgi else None
    lider_rov = filo.find_rov_by_id(lider_id) if lider_id is not None else None
    if lider_rov:
        filo.entity_patlat(lider_rov)


def uygulamayi_calistir(app, rerun_recorder):
    if hasattr(app, "minimap") and app.minimap:
        app.minimap.collider = "box"
    try:
        app.run(interaktif=True)
    except KeyboardInterrupt:
        print("\n🛑 Simülasyon durduruldu.")
    finally:
        rerun_recorder.stop_if_recording()
        try:
            Profiler.rapor_ver()
        except Exception:
            pass
        os.system("stty sane")
        os._exit(0)
