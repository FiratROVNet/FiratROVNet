"""
Minimap üzerinden koordinat seçimi — UI ↔ simülasyon köprüsü.

Simülasyon penceresinde minimapa tıklanınca alan/nokta koordinatları
UI/_minimap_secim.json dosyasına yazılır; bu modül sonucu okur ve alanları doldurur.
"""
from __future__ import annotations

from PyQt5.QtCore import QTimer, QObject
from PyQt5.QtWidgets import QPushButton, QWidget

from UI.kopru import (
    komut_gonder,
    minimap_secim_oku,
    minimap_secim_baslat,
    minimap_secim_iptal,
    minimap_secim_mod_kapat,
)
from UI.tema import METiN, METiN_KOYU, VURGU, PANEL_KENAR


def alan_secim_deger_yaz(deger_yaz, d: dict):
    """Çokgen sonucunu UI alan alanlarına yazar; kutu seçilen alanın merkezine hizalanır."""
    cx = float(d.get("merkez_x", d["x_min"]))
    cy = float(d.get("merkez_y", d["y_min"]))
    x_min, x_max = float(d["x_min"]), float(d["x_max"])
    y_min, y_max = float(d["y_min"]), float(d["y_max"])
    half_w = max(20.0, (x_max - x_min) / 2.0)
    half_h = max(20.0, (y_max - y_min) / 2.0)
    deger_yaz(cx - half_w, cy - half_h, cx + half_w, cy + half_h)


_BTN_CSS = f"""
    QPushButton {{
        color: {METiN};
        background: #111821;
        border: 1px solid {PANEL_KENAR};
        border-radius: 4px;
        padding: 4px 10px;
        font-family: Consolas;
        font-size: 8pt;
    }}
    QPushButton:hover {{
        color: {VURGU};
        border-color: {VURGU};
    }}
    QPushButton:checked {{
        color: #ffee00;
        border-color: #ffee00;
        background: #1a1500;
    }}
"""


class MinimapSecimYardimcisi(QObject):
    """Tek aktif seçim oturumu; QTimer ile JSON sonucunu izler."""

    _ornek: "MinimapSecimYardimcisi | None" = None

    def __init__(self, parent=None):
        super().__init__(parent)
        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._poll)
        self._callback = None
        self._sinyal = None
        self._bitis_fnleri: list = []

    @classmethod
    def _aktif(cls) -> "MinimapSecimYardimcisi":
        if cls._ornek is None:
            cls._ornek = MinimapSecimYardimcisi()
        return cls._ornek

    @classmethod
    def baslat_alan(cls, deger_yaz, sinyal=None, parent=None):
        """deger_yaz(x_min, y_min, x_max, y_max) — çokgen; merkez JSON'da merkez_x/y."""
        def _cb(d: dict):
            alan_secim_deger_yaz(deger_yaz, d)
        cls._baslat("alan", _cb, sinyal, parent)

    @classmethod
    def baslat_nokta(cls, deger_yaz, sinyal=None, parent=None):
        """deger_yaz(x, y) — tek tıklama (Z alanları değişmez)."""
        def _cb(d: dict):
            deger_yaz(float(d["x"]), float(d["y"]))
        cls._baslat("nokta", _cb, sinyal, parent)

    @classmethod
    def _baslat(cls, mod: str, callback, sinyal=None, parent=None, serit_araligi: float = 15.0):
        yardimci = cls._aktif()
        yardimci._durdur(sessiz=True)
        yardimci._callback = callback
        yardimci._sinyal = sinyal
        minimap_secim_baslat(mod, serit_araligi=serit_araligi)
        if sinyal is not None:
            if mod == "alan":
                mesaj = (
                    "Minimap: köşeleri tıklayın, kapatmak için 1. noktaya tekrar tıklayın "
                    "(A* kapalı)"
                )
            else:
                mesaj = "Minimap: hedef noktaya tıklayın (A* kapalı)"
            sinyal.durum_guncellendi.emit(mesaj, "warn")
        yardimci._timer.start()

    @classmethod
    def iptal(cls):
        cls._aktif()._durdur(iptal_et=True)

    def _durdur(self, sessiz: bool = False, iptal_et: bool = False):
        self._timer.stop()
        self._callback = None
        if iptal_et:
            minimap_secim_iptal(gorev_gorselini_temizle=False)
        else:
            minimap_secim_mod_kapat()
        for fn in self._bitis_fnleri:
            try:
                fn()
            except Exception:
                pass
        self._bitis_fnleri.clear()
        if not sessiz and self._sinyal is not None:
            self._sinyal.durum_guncellendi.emit("Harita seçimi iptal edildi.", "ok")

    def _poll(self):
        d = minimap_secim_oku()
        if not d.get("aktif") and not d.get("tamamlandi") and not d.get("iptal"):
            return
        mesaj = d.get("mesaj")
        if mesaj and self._sinyal is not None and d.get("aktif"):
            self._sinyal.durum_guncellendi.emit(str(mesaj), "warn")
        # tamamlandi önce — ardından gelen iptal JSON'u alan çizimini silmesin
        if d.get("tamamlandi") and self._callback:
            cb = self._callback
            self._callback = None
            self._timer.stop()
            cb(d)
            for fn in self._bitis_fnleri:
                try:
                    fn()
                except Exception:
                    pass
            self._bitis_fnleri.clear()
            if self._sinyal is not None:
                merkez = ""
                if d.get("mod") == "alan" and "merkez_x" in d:
                    merkez = f" | Merkez: ({d['merkez_x']:.1f}, {d['merkez_y']:.1f})"
                self._sinyal.durum_guncellendi.emit(
                    f"Haritadan koordinat alındı.{merkez}", "ok"
                )
            return
        if d.get("iptal"):
            self._timer.stop()
            self._callback = None
            # Sim zaten iptal komutunu uyguladı; tekrar kuyruğa iptal gönderme
            for fn in self._bitis_fnleri:
                try:
                    fn()
                except Exception:
                    pass
            self._bitis_fnleri.clear()
            if self._sinyal is not None:
                self._sinyal.durum_guncellendi.emit("Harita seçimi iptal edildi.", "ok")

    def bitiste(self, fn):
        self._bitis_fnleri.append(fn)


def haritadan_sec_butonu(
    mod: str,
    deger_yaz,
    sinyal=None,
    metin: str = "🗺  Haritadan Seç",
    parent: QWidget | None = None,
    serit_araligi_al=None,
) -> QPushButton:
    """Alan veya nokta seçimi başlatır."""
    btn = QPushButton(metin, parent)
    btn.setToolTip(
        "Simülasyon minimapında tıklayın. Alan: çokgen köşeleri, kapatmak için 1. noktaya "
        "tekrar tıklayın (tarama kutusu + merkez otomatik). Nokta: tek tık. Esc = iptal. "
        "Seçim sırasında A* navigasyon kapalıdır."
    )
    btn.setStyleSheet(_BTN_CSS)

    def _baslat():
        btn.setEnabled(False)
        btn.setText("⏳ Minimap…")
        yardimci = MinimapSecimYardimcisi._aktif()

        def _btn_sifirla():
            btn.setEnabled(True)
            btn.setText(metin)

        yardimci.bitiste(_btn_sifirla)
        serit = 15.0
        if serit_araligi_al is not None:
            try:
                serit = float(serit_araligi_al())
            except Exception:
                serit = 15.0
        if mod == "alan":
            def _alan_cb(d):
                alan_secim_deger_yaz(deger_yaz, d)
            MinimapSecimYardimcisi._baslat(
                "alan", _alan_cb, sinyal, parent, serit_araligi=serit
            )
        else:
            MinimapSecimYardimcisi.baslat_nokta(deger_yaz, sinyal=sinyal, parent=parent)

    btn.clicked.connect(_baslat)
    return btn
