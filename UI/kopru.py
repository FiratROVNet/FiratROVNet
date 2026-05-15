"""
FiratROVNet Komuta Arayüzü — Simülasyon Köprüsü

Simülasyon çalışıyorsa filo nesnesine bağlanır ve komutları doğrudan iletir.
Simülasyon çalışmıyorsa komutları KOMUT_KUYRUĞU.txt dosyasına yazar
(simülasyon başladığında okuyabilir).
"""

from __future__ import annotations
import os
import sys
import json
import threading
import time
from typing import Any

# ── Simülasyon bağlantısı ─────────────────────────────────────────────────────
_filo_ref = None          # Filo nesnesi (aynı işlemde çalışıyorsa)
_bagli   = False

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KUYRUK_DOSYA = os.path.join(_ROOT, "KOMUT_KUYRUĞU.txt")
DURUM_DOSYA  = os.path.join(_ROOT, "UI", "_rov_durumu.json")

# Durum dosyası önbelleği
_durum_cache: dict = {}
_durum_cache_ts: float = 0.0
_CACHE_SURE = 2.0   # saniye — bu süreden eski veri "bağlantı yok" sayılır
_DURUM_OKU_CACHE_SURE = 0.35  # hizli UI geri bildirimi icin kisa onbellek


def filo_bagla(filo):
    """Ana simülasyondan çağrılır: `from UI.kopru import filo_bagla; filo_bagla(filo)`"""
    global _filo_ref, _bagli
    _filo_ref = filo
    _bagli    = True


def _durum_oku() -> dict:
    """Durum dosyasını okur; önbellekten döner."""
    global _durum_cache, _durum_cache_ts
    now = time.monotonic()
    if now - _durum_cache_ts < _DURUM_OKU_CACHE_SURE:
        return _durum_cache
    try:
        with open(DURUM_DOSYA, "r", encoding="utf-8") as f:
            _durum_cache = json.load(f)
        _durum_cache_ts = now
    except Exception:
        pass
    return _durum_cache


def sim_bagli_mi() -> bool:
    """Durum dosyası 2 saniyeden tazeyse simülasyon bağlı sayılır."""
    if bagli_mi():
        return True
    d = _durum_oku()
    ts = d.get("timestamp", 0)
    return (time.time() - ts) < _CACHE_SURE


def bagli_mi() -> bool:
    return _bagli and _filo_ref is not None


# ── Komut Yürütücü ────────────────────────────────────────────────────────────

def _calistir(komut: str) -> str:
    """
    Komutu çalıştırır.
    - Bağlı: filo üzerinden doğrudan exec()
    - Bağlı değil: dosyaya yazar
    Sonucu string olarak döner.
    """
    if bagli_mi():
        try:
            local_ns = {"filo": _filo_ref}
            exec(komut, local_ns)          # noqa: S102
            return f"✔ {komut}"
        except Exception as exc:
            return f"✗ HATA: {exc}"
    else:
        try:
            with open(KUYRUK_DOSYA, "a", encoding="utf-8") as f:
                f.write(komut + "\n")
            return f"📋 Kuyruğa eklendi: {komut}"
        except Exception as exc:
            return f"✗ Dosya yazma hatası: {exc}"


def komut_gonder(komut: str, callback=None):
    """Arka planda komut gönderir; sonucu callback(str) ile iletir."""
    def _run():
        sonuc = _calistir(komut)
        if callback:
            callback(sonuc)
    threading.Thread(target=_run, daemon=True).start()


# ── Simülasyon Durum Sorgu ────────────────────────────────────────────────────

def rov_listesi() -> list[dict]:
    """Mevcut ROV bilgilerini döner. [{id, rol, gorev, gps, gat_kodu, batarya, hiz}, ...]"""
    if bagli_mi():
        # Aynı process: doğrudan filo
        try:
            sonuc = []
            for rov in _filo_ref.rovs:
                gps   = _filo_ref.get(rov.id, "gps") or (0, 0, 0)
                gorev = getattr(getattr(rov, "gnc", None), "gorev", "idle") or "idle"
                sonuc.append({
                    "id":       rov.id,
                    "rol":      getattr(rov, "role", 0),
                    "gorev":    gorev,
                    "gps":      tuple(float(v) for v in gps),
                    "gat_kodu": int(getattr(rov, "gat_kodu", 0)),
                    "batarya":  round(float(getattr(rov, "battery", 1.0)), 2),
                    "hiz":      0.0,
                    "grup_id":  int(getattr(rov, "group_id", 0)),
                })
            return sonuc
        except Exception:
            return []

    # Subprocess: durum dosyasından oku (timestamp kontrolü ile)
    d = _durum_oku()
    if sim_bagli_mi() and d.get("rovlar"):
        return [
            {
                "id":       r["id"],
                "rol":      r.get("rol", 0),
                "gorev":    r.get("gorev", "idle"),
                "gps":      tuple(r.get("gps", [0, 0, 0])),
                "gat_kodu": r.get("gat_kodu", 0),
                "batarya":  r.get("batarya", 1.0),
                "hiz":      r.get("hiz", 0.0),
                "grup_id":  r.get("grup_id", 0),
            }
            for r in d["rovlar"]
        ]

    # Simülasyon bağlı değil — boş liste döndür
    return []


def grup_bilgisi() -> dict[int, list[int]]:
    """Grup → ROV ID listesi eşlemesini döner."""
    if bagli_mi():
        try:
            sonuc: dict[int, list[int]] = {}
            for g_id, rovlar in _filo_ref.g_rovs.items():
                sonuc[int(g_id)] = [int(r.id) for r in rovlar if r]
            return sonuc
        except Exception:
            return {}

    # Subprocess: durum dosyasından oku (timestamp kontrolü ile)
    d = _durum_oku()
    if sim_bagli_mi() and d.get("gruplar"):
        return {int(k): v for k, v in d["gruplar"].items()}

    return {}


def aktif_gorevler_bilgisi() -> dict:
    """Aktif görev durumlarını döner. Sadece aynı-process bağlantıda çalışır."""
    if not bagli_mi():
        return {}
    try:
        sonuc: dict = {}

        # Alan Tarama
        at = getattr(_filo_ref, "alan_tarama_gorevi", None)
        if at and getattr(at, "aktif_planlar", None):
            planlar: dict = {}
            for g_id, plan in at.aktif_planlar.items():
                alan = getattr(plan, "alan", None)
                planlar[int(g_id)] = {
                    "lider_id":   plan.lider_id,
                    "rov_sayisi": len(plan.rota_by_rov),
                    "derinlik":   round(float(plan.derinlik), 1),
                    "alan": (
                        getattr(alan, "x_min", 0), getattr(alan, "y_min", 0),
                        getattr(alan, "x_max", 0), getattr(alan, "y_max", 0),
                    ) if alan else None,
                    "gorev_adi":  getattr(plan, "gorev_adi", "alan_tarama"),
                }
            if planlar:
                sonuc["alan_tarama"] = planlar

        # Arama Kurtarma
        ak = getattr(_filo_ref, "arama_kurtarma_gorevi", None)
        if ak and getattr(ak, "aktif_plan", None):
            plan = ak.aktif_plan
            sonuc["arama_kurtarma"] = {
                "grup_id":        int(plan.grup_id),
                "rov_sayisi":     len(plan.rota_by_rov),
                "hedef_siniflari": sorted(getattr(ak, "hedef_siniflari", [])),
            }

        # İmha
        imha = getattr(_filo_ref, "imha_gorevi", None)
        if imha and (getattr(imha, "hedef", None) is not None
                     or getattr(imha, "gorevli_rov_id", None) is not None):
            h = getattr(imha, "hedef", None)
            sonuc["imha"] = {
                "hedef":          tuple(float(v) for v in h) if h else None,
                "gorevli_rov_id": getattr(imha, "gorevli_rov_id", None),
            }

        return sonuc
    except Exception:
        return {}
