import math
import os
import json
import subprocess
import sys
import threading
import time
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
    def grup_degistir(rov_id, group_id, role=None, mod=None):
        rov = filo.find_rov_by_id(int(rov_id))
        if rov is None:
            print(f"⚠️ ROV-{rov_id} bulunamadı.")
            return False
        if group_id is None or (isinstance(group_id, str) and str(group_id).strip().lower() in ("", "none")):
            return filo.rov_usye_al(int(rov_id))
        yeni_grup = int(group_id)
        rov.group_id = yeni_grup
        if role is not None:
            rov.role = int(role)
            if hasattr(rov, "_etiket_guncelle"):
                rov._etiket_guncelle()
        if mod is not None and getattr(rov, "gnc", None) is not None:
            rov.gnc.mod = int(mod)
        if yeni_grup >= 0:
            if not isinstance(getattr(filo, "aktif_formasyon", None), dict):
                filo.aktif_formasyon = {}
            filo.aktif_formasyon.setdefault(
                yeni_grup,
                {"id": "LINE", "aralik": 10, "is_3d": False, "yaw": 0, "g_id": yeni_grup},
            )
        dirty = getattr(app, "mark_ui_state_dirty", None)
        if callable(dirty):
            dirty()
        rol_metni = f", role={getattr(rov, 'role', None)}"
        print(f"✅ ROV-{rov.id} Grup-{yeni_grup} olarak güncellendi{rol_metni}.")
        return True

    app.konsola_ekle("git", lambda rov_id, x, z, y=None, ai=True: filo.git(rov_id, x, z, y, ai))
    app.konsola_ekle("move", lambda rov_id, yon, guc=1.0: filo.move(rov_id, yon, guc))
    app.konsola_ekle("get", lambda rov_id, veri_tipi: filo.get(rov_id, veri_tipi))
    app.konsola_ekle("set", lambda rov_id, ayar_adi, deger: filo.set(rov_id, ayar_adi, deger))
    app.konsola_ekle("grup_degistir", grup_degistir)
    app.konsola_ekle("rov_usye_al", filo.rov_usye_al)
    app.konsola_ekle("Ada", lambda ada_id, x=None, y=None: app.Ada(ada_id, x, y))
    app.konsola_ekle("ROV", lambda rov_id, x=None, y=None, z=None: app.ROV(rov_id, x, y, z))
    app.konsola_ekle("filo", filo)
    app.konsola_ekle("panels", getattr(filo, "panels", None))
    app.konsola_ekle("rovs", app.rovs)
    app.konsola_ekle("cfg", cfg)
    app.konsola_ekle("nav_queue", filo.nav_queue)


class KomutaArayuzu:
    def __init__(self, script_dir, app, filo):
        self.script_dir = script_dir
        self.app = app
        self.filo = filo
        self.durum_dosya = os.path.join(script_dir, "UI", "_rov_durumu.json")
        self.kuyruk_dosya = os.path.join(script_dir, "KOMUT_KUYRUĞU.txt")
        self.proc = None
        self.dirty_until = 0.0
        setattr(app, "mark_ui_state_dirty", self.mark_dirty)

    def mark_dirty(self, burst_sure=1.2):
        hedef = time.monotonic() + max(0.2, float(burst_sure))
        if hedef > self.dirty_until:
            self.dirty_until = hedef

    def open(self):
        if self.proc is not None and self.proc.poll() is None:
            print("🖥️ Komuta Arayüzü zaten açık.")
            return

        env = os.environ.copy()
        try:
            from PyQt5.QtCore import QLibraryInfo

            qt_plugins = QLibraryInfo.location(QLibraryInfo.PluginsPath)
            if os.path.isdir(qt_plugins):
                env["QT_QPA_PLATFORM_PLUGIN_PATH"] = qt_plugins
                env["QT_PLUGIN_PATH"] = qt_plugins
        except Exception:
            env.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
            env.pop("QT_PLUGIN_PATH", None)
        env.pop("QT_QPA_PLATFORMTHEME", None)

        script = os.path.join(self.script_dir, "UI", "baslat.py")
        self.proc = subprocess.Popen([sys.executable, script], cwd=self.script_dir, env=env)
        print(f"🖥️ Komuta Arayüzü açıldı (PID: {self.proc.pid}).")

    def close(self):
        if self.proc is None or self.proc.poll() is not None:
            return
        self.proc.terminate()
        print("🖥️ Komuta Arayüzü kapatıldı.")

    def toggle(self):
        if self.proc is not None and self.proc.poll() is None:
            self.close()
            return
        self.open()

    def rov_ekle(self, group_id=None, position=None, x=None, y=None, z=None, model_key=None, rol=0, role=None):
        from FiratROVNet.simulasyon import ROV

        group_id = self._opsiyonel_int(group_id)
        rol_degeri = self._opsiyonel_int(role if role is not None else rol)
        if rol_degeri is None:
            rol_degeri = 0
        if position is None:
            spawn_al = getattr(self.app, "ileri_karakol_spawn_pozisyonu", None)
            if x is None and y is None and z is None and callable(spawn_al):
                position = spawn_al(rastgele=True)
            else:
                position = (
                    float(0 if x is None else x),
                    float(-10 if y is None else y),
                    float(0 if z is None else z),
                )
        model = model_key or getattr(self.app, "rov_model", "submarine")
        rov = ROV(
            group_id=group_id,
            position=position,
            loader_ref=self.app.loader,
            model_key=model,
            rol=rol_degeri,
        )
        sonuc = rov.ekle(self.app)
        self.mark_dirty()
        return sonuc

    def rov_cikar(self, rov_id):
        rov = self.filo.find_rov_by_id(int(rov_id))
        if rov is None:
            return False
        sonuc = rov.cikar()
        self.mark_dirty()
        return sonuc

    def durum_yaz(self):
        try:
            rovlar = []
            for rov in self.filo.rovs:
                try:
                    gps = self.filo.get(rov.id, "gps") or (0, 0, 0)
                    gorev = getattr(getattr(rov, "gnc", None), "gorev", "idle") or "idle"
                    batarya = float(getattr(rov, "battery", 1.0))
                    hiz_kaynak = getattr(rov, "velocity", None)
                    hiz = float(hiz_kaynak.length() if hasattr(hiz_kaynak, "length") else 0.0)
                    rovlar.append(
                        {
                            "id": int(rov.id),
                            "rol": int(getattr(rov, "role", 0)),
                            "gorev": str(gorev),
                            "gps": [round(float(v), 1) for v in gps],
                            "gat_kodu": int(getattr(rov, "gat_kodu", 0)),
                            "batarya": round(batarya, 2),
                            "hiz": round(hiz, 2),
                            "grup_id": getattr(rov, "group_id", None),
                        }
                    )
                except Exception:
                    pass

            gruplar = {}
            for g_id, grup in self.filo.g_rovs.items():
                if g_id is None:
                    continue
                aktif = [
                    int(rov.id)
                    for rov in (grup or [])
                    if rov and not getattr(rov, "is_destroyed", False)
                ]
                if aktif:
                    gruplar[str(int(g_id))] = aktif

            veri = {"timestamp": time.time(), "rovlar": rovlar, "gruplar": gruplar, "bagli": True}
            os.makedirs(os.path.dirname(self.durum_dosya), exist_ok=True)
            tmp = self.durum_dosya + ".tmp"
            with open(tmp, "w", encoding="utf-8") as dosya:
                json.dump(veri, dosya, ensure_ascii=False)
            os.replace(tmp, self.durum_dosya)
        except Exception as exc:
            print(f"[UI-Durum] yazma hatası: {exc}")

    def komut_oku(self):
        try:
            if not os.path.exists(self.kuyruk_dosya):
                return
            with open(self.kuyruk_dosya, "r", encoding="utf-8") as dosya:
                satirlar = [satir.strip() for satir in dosya.readlines() if satir.strip()]
            if not satirlar:
                return
            open(self.kuyruk_dosya, "w", encoding="utf-8").close()

            from FiratROVNet.simulasyon import ROV

            local_ns = {
                "app": self.app,
                "filo": self.filo,
                "ROV": ROV,
                "ui_rov_ekle": self.rov_ekle,
                "ui_rov_cikar": self.rov_cikar,
                "ui_minimap_secim_baslat": ui_minimap_secim_baslat,
                "ui_minimap_secim_iptal": ui_minimap_secim_iptal,
                "ui_minimap_secim_mod_kapat": ui_minimap_secim_mod_kapat,
                "ui_minimap_gorev_alan_temizle": ui_minimap_gorev_alan_temizle,
                "grup_degistir": getattr(self.app, "konsol_verileri", {}).get("grup_degistir"),
            }
            yasakli_oruntuler = (
                "Ortam(",
                "Ursina(",
                "sim_olustur(",
                "__import__",
                "import ",
                "from ",
            )
            for komut in satirlar:
                try:
                    if any(oruntu in komut for oruntu in yasakli_oruntuler):
                        print(f"[UI-Komut] ⛔ engellendi: {komut}")
                        continue
                    exec(komut, local_ns)  # noqa: S102
                    self.mark_dirty()
                    print(f"[UI-Komut] ✔ {komut}")
                except Exception as exc:
                    print(f"[UI-Komut] ✗ {komut} -> {exc}")
        except Exception as exc:
            print(f"[UI-Komut] okuma hatası: {exc}")

    def guncelle(self, scheduler, dt):
        yazildi = False
        if scheduler.due("ui_durum_hizli", 2.0, dt, first=False) and time.monotonic() < self.dirty_until:
            self.durum_yaz()
            yazildi = True
        if not yazildi and scheduler.due("ui_durum", 1.0, dt):
            self.durum_yaz()
        if scheduler.due("ui_komut", 2.0, dt):
            self.komut_oku()

    @staticmethod
    def _opsiyonel_int(deger):
        if deger is None:
            return None
        if isinstance(deger, str) and deger.strip().lower() in {"", "none", "null"}:
            return None
        return int(deger)


def tahminler_al(app, tahminler_cache):
    filo = getattr(app, "filo", None)
    if filo is not None and hasattr(filo, "rovs"):
        rov_sayisi = len(filo.rovs)
    else:
        rov_sayisi = len([r for r in getattr(app, "rovs", []) if r and not getattr(r, "is_destroyed", False)])
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
            if g_id is None:
                continue
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


def aktif_rov_idleri(filo):
    try:
        ids = [
            int(getattr(rov, "id"))
            for rov in (filo.rovs or [])
            if rov and not getattr(rov, "is_destroyed", False)
        ]
        return sorted(ids)
    except Exception:
        return []


def sac_hud_toggle(filo, sac_hud, bilgi_rov_id):
    if not filo.rovs:
        print("ℹ️ SAC paneli için aktif ROV yok.")
        return
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
    ids = aktif_rov_idleri(filo)
    if not ids:
        print("ℹ️ Aktif ROV yok. Runtime'da ROV ekledikten sonra tekrar deneyin.")
        return bilgi_rov_id
    if bilgi_rov_id in ids:
        bilgi_rov_id = ids[(ids.index(bilgi_rov_id) + 1) % len(ids)]
    else:
        bilgi_rov_id = ids[0]
    filo.kamera_ayarla(rov_id=bilgi_rov_id)
    print(f"🔄 Aktif ROV: {bilgi_rov_id}")
    return bilgi_rov_id


def minimap_secim_dosya_yolu(app) -> str:
    script_dir = getattr(app, "_firat_script_dir", None)
    if not script_dir:
        script_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(script_dir, "UI", "_minimap_secim.json")


def _minimap_secim_yaz(app, payload: dict):
    yol = minimap_secim_dosya_yolu(app)
    payload = {**payload, "timestamp": time.time()}
    os.makedirs(os.path.dirname(yol), exist_ok=True)
    tmp = yol + ".tmp"
    with open(tmp, "w", encoding="utf-8") as dosya:
        json.dump(payload, dosya, ensure_ascii=False)
    os.replace(tmp, yol)


def ui_minimap_secim_aktif(app) -> bool:
    """UI harita seçim modu açıkken minimap tıklaması navigasyon/A* tetiklemez."""
    return getattr(app, "_ui_minimap_picker", None) is not None


def _poligon_kapat_yakin_mi(yeni: tuple[float, float], ilk: tuple[float, float], n_kose: int, havuz_gen: float) -> bool:
    if n_kose < 3:
        return False
    esik = max(12.0, float(havuz_gen) * 0.06)
    return math.hypot(yeni[0] - ilk[0], yeni[1] - ilk[1]) <= esik


def _poligon_ozet(noktalar: list) -> tuple[float, float, float, float, float, float]:
    xs = [float(p[0]) for p in noktalar]
    ys = [float(p[1]) for p in noktalar]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    return x_min, y_min, x_max, y_max, cx, cy


def minimap_tiklama_to_sim(app, local_pos) -> tuple[float, float] | None:
    """Minimap yerel tıklamayı simülasyon X,Y (düzlem) koordinatına çevirir."""
    if local_pos is None:
        return None
    havuz = float(getattr(app, "havuz_genisligi", 200) or 200)
    olcek = havuz * 2.0
    return float(local_pos.x) * olcek, float(local_pos.y) * olcek


def ui_minimap_secim_baslat(app, mod: str = "alan", serit_araligi: float = 15.0):
    mod = "nokta" if str(mod).strip().lower() == "nokta" else "alan"
    serit = max(1.0, float(serit_araligi or 15.0))
    setattr(app, "_ui_minimap_serit_araligi", serit)
    app._ui_minimap_picker = {"mod": mod, "noktalar": [], "serit_araligi": serit}
    mm = getattr(app, "minimap", None)
    if mm is not None:
        mm.visible = True
        mm.update_path([])
        if mod == "alan" and hasattr(mm, "alan_secim_baslat"):
            mm.alan_secim_baslat()
    mesaj = (
        "Minimap: köşeleri tıklayın, kapatmak için 1. noktaya tekrar tıklayın (A* kapalı)"
        if mod == "alan"
        else "Minimap: hedef noktaya sol tıklayın (A* kapalı)"
    )
    _minimap_secim_yaz(
        app,
        {
            "aktif": True,
            "tamamlandi": False,
            "iptal": False,
            "mod": mod,
            "noktalar": [],
            "mesaj": mesaj,
        },
    )
    print(f"🗺️ [UI] Haritadan seçim: {mesaj} (Esc = iptal)")


def ui_minimap_gorev_alan_temizle(app):
    """Aktif görev minimap alan çizimini kaldırır."""
    mm = getattr(app, "minimap", None)
    if mm is not None and hasattr(mm, "alan_gorev_temizle"):
        mm.alan_gorev_temizle()
    setattr(app, "_ui_minimap_gorev_poligon", None)


def ui_minimap_secim_mod_kapat(app):
    """Seçim modunu kapatır; tamamlanmış alan çizimine dokunmaz (JSON iptal yazmaz)."""
    app._ui_minimap_picker = None
    mm = getattr(app, "minimap", None)
    if mm is not None and hasattr(mm, "alan_secim_gecici_temizle"):
        mm.alan_secim_gecici_temizle()
    poligon = getattr(app, "_ui_minimap_gorev_poligon", None)
    if mm is not None and poligon and hasattr(mm, "alan_gorev_goster"):
        serit = float(getattr(app, "_ui_minimap_serit_araligi", 15.0) or 15.0)
        mm.alan_gorev_goster(poligon, serit_araligi=serit)


def ui_minimap_secim_iptal(app, gorev_gorselini_temizle: bool = False):
    app._ui_minimap_picker = None
    mm = getattr(app, "minimap", None)
    if mm is not None:
        if gorev_gorselini_temizle:
            if hasattr(mm, "alan_secim_temizle"):
                mm.alan_secim_temizle()
        elif hasattr(mm, "alan_secim_gecici_temizle"):
            mm.alan_secim_gecici_temizle()
        mm.update_path([])
    if gorev_gorselini_temizle:
        ui_minimap_gorev_alan_temizle(app)
    _minimap_secim_yaz(
        app,
        {
            "aktif": False,
            "tamamlandi": False,
            "iptal": True,
            "mod": None,
            "noktalar": [],
            "mesaj": "Seçim iptal edildi.",
        },
    )
    print("🗺️ [UI] Haritadan seçim iptal edildi.")


def minimap_ui_secim_isle(app, filo, bilgi_rov_id) -> bool:
    """Aktif UI seçim modundaysa tıklamayı işler ve True döner."""
    picker = getattr(app, "_ui_minimap_picker", None)
    if not picker or not (hasattr(app, "minimap") and mouse.hovered_entity == app.minimap):
        return False

    sim_xy = minimap_tiklama_to_sim(app, mouse.point)
    if sim_xy is None:
        return True

    sx, sy = sim_xy
    mod = picker.get("mod", "alan")
    mm = getattr(app, "minimap", None)

    if mod == "nokta":
        app._ui_minimap_picker = None
        _minimap_secim_yaz(
            app,
            {
                "aktif": False,
                "tamamlandi": True,
                "iptal": False,
                "mod": "nokta",
                "x": sx,
                "y": sy,
                "merkez_x": sx,
                "merkez_y": sy,
                "noktalar": [[sx, sy]],
                "mesaj": f"Nokta: ({sx:.1f}, {sy:.1f})",
            },
        )
        print(f"🗺️ [UI] Seçilen nokta: X={sx:.1f}  Y={sy:.1f}")
        return True

    # ── Çokgen alan: köşe ekle veya 1. noktaya dönünce kapat ──
    havuz = float(getattr(app, "havuz_genisligi", 200) or 200)
    mevcut = list(picker.get("noktalar") or [])

    if len(mevcut) >= 3 and _poligon_kapat_yakin_mi((sx, sy), tuple(mevcut[0]), len(mevcut), havuz):
        poligon = mevcut
        x_min, y_min, x_max, y_max, cx, cy = _poligon_ozet(poligon)
        app._ui_minimap_picker = None
        if mm is not None and hasattr(mm, "alan_gorev_goster"):
            serit = float(picker.get("serit_araligi", getattr(app, "_ui_minimap_serit_araligi", 15.0)) or 15.0)
            mm.alan_gorev_goster(poligon, serit_araligi=serit)
            setattr(app, "_ui_minimap_gorev_poligon", poligon)
            setattr(app, "_ui_minimap_serit_araligi", serit)
        _minimap_secim_yaz(
            app,
            {
                "aktif": False,
                "tamamlandi": True,
                "iptal": False,
                "mod": "alan",
                "x_min": x_min,
                "y_min": y_min,
                "x_max": x_max,
                "y_max": y_max,
                "merkez_x": cx,
                "merkez_y": cy,
                "noktalar": poligon,
                "mesaj": (
                    f"Alan merkezi: ({cx:.1f}, {cy:.1f}) | "
                    f"Tarama kutusu: ({x_min:.0f},{y_min:.0f})–({x_max:.0f},{y_max:.0f})"
                ),
            },
        )
        print(
            f"🗺️ [UI] Alan seçildi | merkez=({cx:.1f},{cy:.1f}) | "
            f"kutu=[{x_min:.1f}–{x_max:.1f}]×[{y_min:.1f}–{y_max:.1f}]"
        )
        return True

    mevcut.append([round(sx, 1), round(sy, 1)])
    picker["noktalar"] = mevcut
    if mm is not None and hasattr(mm, "alan_secim_ciz"):
        mm.alan_secim_ciz(mevcut, kapatildi=False)

    if len(mevcut) == 1:
        mesaj = f"Köşe 1: ({sx:.1f}, {sy:.1f}) — diğer köşeleri tıklayın"
    elif len(mevcut) == 2:
        mesaj = f"Köşe {len(mevcut)} eklendi — en az 3 köşe, sonra 1. noktaya tıklayarak kapatın"
    else:
        mesaj = (
            f"Köşe {len(mevcut)} eklendi — alanı kapatmak için "
            f"1. noktaya ({mevcut[0][0]:.0f}, {mevcut[0][1]:.0f}) tekrar tıklayın"
        )
    _minimap_secim_yaz(
        app,
        {
            "aktif": True,
            "tamamlandi": False,
            "iptal": False,
            "mod": "alan",
            "noktalar": mevcut,
            "mesaj": mesaj,
        },
    )
    print(f"🗺️ [UI] Köşe {len(mevcut)}: ({sx:.1f}, {sy:.1f})")
    return True


def minimap_tiklama_isle(app, filo, bilgi_rov_id):
    """Sol tık: önce UI koordinat seçimi, yoksa normal navigasyon hedefi."""
    if minimap_ui_secim_isle(app, filo, bilgi_rov_id):
        dirty = getattr(app, "mark_ui_state_dirty", None)
        if callable(dirty):
            dirty()
        return
    minimap_hedef_ata(app, filo, bilgi_rov_id)


def minimap_hedef_ata(app, filo, bilgi_rov_id):
    if ui_minimap_secim_aktif(app):
        return
    if not (hasattr(app, "minimap") and mouse.hovered_entity == app.minimap):
        return
    sim_xy = minimap_tiklama_to_sim(app, mouse.point)
    if sim_xy is None:
        return
    sim_x, sim_y = sim_xy
    mevcut_z = -20
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
    rov = filo.find_rov_by_id(bilgi_rov_id)
    if rov is None:
        print("ℹ️ Patlatılacak lider yok; aktif ROV bulunmuyor.")
        return
    lider_bilgi = filo.find_leader_info(g_id=rov.group_id)
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
