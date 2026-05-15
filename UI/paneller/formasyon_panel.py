"""
Formasyon Paneli — Formasyon tipi, aralık ve grup seçimi ile uygulama.
"""
from __future__ import annotations

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QSpinBox, QDoubleSpinBox, QPushButton, QGridLayout,
    QCheckBox, QFrame, QComboBox,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont

from UI.tema import (
    VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI,
    PANEL, PANEL_KENAR, ARKA_PLAN, BUTON_NORMAL,
)
from UI.kopru import komut_gonder, seviye_tespit


# (isim, ikon, açıklama)
FORMASYONLAR = [
    ("LINE",      "━━━",  "Tek sıra"),
    ("COLUMN",    "║║║",  "Sütun (art arda)"),
    ("V_SHAPE",   " ∨  ", "V şekli (kuş uçuşu)"),
    ("WEDGE",     " ◁  ", "Kama"),
    ("TRIANGLE",  " △  ", "Üçgen"),
    ("CROSS",     " ✚  ", "Haç"),
    ("SPREAD",    "···",  "Yayılım (geniş alan)"),
    ("STAR",      " ✦  ", "Yıldız"),
    ("HEXAGON",   " ⬡  ", "Altıgen"),
    ("WAVE",      "∿∿∿",  "Dalga"),
    ("SPIRAL",    " @  ", "Spiral"),
    ("TSHAPE",    " ⊤  ", "T şekli"),
]


class FormasyonKarti(QPushButton):
    """Tıklanabilir formasyon kartı."""
    def __init__(self, isim: str, ikon: str, aciklama: str, parent=None):
        super().__init__(parent)
        self.formasyon_isim = isim
        self.setCheckable(True)
        self.setFixedHeight(56)
        self.setText(f"{ikon}\n{isim}")
        self.setToolTip(aciklama)
        self.setStyleSheet(f"""
            QPushButton {{
                background-color: {BUTON_NORMAL};
                color: {METiN};
                border: 1px solid {PANEL_KENAR};
                border-radius: 5px;
                font-family: "Consolas";
                font-size: 8pt;
            }}
            QPushButton:checked {{
                background-color: #003049;
                border: 2px solid {VURGU};
                color: {VURGU};
            }}
            QPushButton:hover:!checked {{
                border: 1px solid {VURGU};
                color: {METiN};
            }}
        """)


class FormasyonPanel(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal=None, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        self._secili_kart: FormasyonKarti | None = None

        ana_lay = QVBoxLayout(self)
        ana_lay.setContentsMargins(0, 0, 0, 0)
        ana_lay.setSpacing(8)

        # ── Formasyon ızgarası ───────────────────────────────────────────────
        grid_kutu = QGroupBox("FORMASYON SEÇ")
        grid_lay  = QGridLayout(grid_kutu)
        grid_lay.setSpacing(6)

        self._kartlar: list[FormasyonKarti] = []
        for i, (isim, ikon, aciklama) in enumerate(FORMASYONLAR):
            kart = FormasyonKarti(isim, ikon, aciklama)
            kart.clicked.connect(lambda checked, k=kart: self._kart_sec(k))
            grid_lay.addWidget(kart, i // 4, i % 4)
            self._kartlar.append(kart)

        ana_lay.addWidget(grid_kutu)

        # ── Parametreler ─────────────────────────────────────────────────────
        param_kutu = QGroupBox("FORMASYON PARAMETRELERİ")
        param_lay  = QGridLayout(param_kutu)
        param_lay.setSpacing(8)
        param_lay.setColumnStretch(1, 1)

        # Grup ID
        param_lay.addWidget(self._etiket("Grup ID"), 0, 0)
        self.spin_grup = QSpinBox()
        self.spin_grup.setRange(0, 9)
        self.spin_grup.setToolTip("Formasyonun uygulanacağı grup")
        param_lay.addWidget(self.spin_grup, 0, 1)

        # Aralık
        param_lay.addWidget(self._etiket("Aralık (m)"), 1, 0)
        self.spin_aralik = QDoubleSpinBox()
        self.spin_aralik.setRange(5.0, 100.0)
        self.spin_aralik.setValue(15.0)
        self.spin_aralik.setSingleStep(1.0)
        self.spin_aralik.setToolTip("ROV'lar arasındaki mesafe (metre)")
        param_lay.addWidget(self.spin_aralik, 1, 1)

        # 3D modu
        self.chk_3d = QCheckBox("3D Modu (Z yayılımı)")
        self.chk_3d.setToolTip("Z ekseninde de yayılım yapılır")
        param_lay.addWidget(self.chk_3d, 2, 0, 1, 2)

        # Lider takibi
        self.chk_takip = QCheckBox("Formasyon sonrası lider takibini aç")
        self.chk_takip.setChecked(True)
        self.chk_takip.setToolTip("filo.change_mode(g_id, 1) komutunu otomatik ekler")
        param_lay.addWidget(self.chk_takip, 3, 0, 1, 2)

        ana_lay.addWidget(param_kutu)

        # ── Uygula butonu ────────────────────────────────────────────────────
        self.btn_uygula = QPushButton("⬡  Formasyonu Uygula")
        self.btn_uygula.setObjectName("btn_basla")
        self.btn_uygula.clicked.connect(self._uygula)
        self.btn_uygula.setEnabled(False)
        ana_lay.addWidget(self.btn_uygula)

        # Seçili formasyon göstergesi
        self.lbl_secili = QLabel("Formasyon seçilmedi")
        self.lbl_secili.setAlignment(Qt.AlignCenter)
        self.lbl_secili.setStyleSheet(f"color: {METiN_KOYU}; font-size: 8pt;")
        ana_lay.addWidget(self.lbl_secili)

        ana_lay.addStretch()

    @staticmethod
    def _etiket(metin: str) -> QLabel:
        lbl = QLabel(metin)
        lbl.setStyleSheet(f"color: {METiN_KOYU}; font-size: 9pt;")
        return lbl

    def _kart_sec(self, kart: FormasyonKarti):
        # Öncekini kaldır
        if self._secili_kart and self._secili_kart is not kart:
            self._secili_kart.setChecked(False)
        self._secili_kart = kart
        kart.setChecked(True)
        self.btn_uygula.setEnabled(True)
        self.lbl_secili.setText(f"Seçili: {kart.formasyon_isim}")
        self.lbl_secili.setStyleSheet(f"color: {VURGU}; font-size: 9pt; font-weight: bold;")

    def _uygula(self):
        if self._secili_kart is None:
            return
        isim   = self._secili_kart.formasyon_isim
        aralik = self.spin_aralik.value()
        g_id   = self.spin_grup.value()
        is_3d  = self.chk_3d.isChecked()

        # formasyon() g_id desteklemiyor; önce formasyon_sec ile grubu aktifleştir,
        # ardından formasyon() çağır (aktif grubun liderine uygular)
        komutlar = []
        k_sec = f"filo.formasyon_sec(g_id={g_id})"
        komutlar.append((k_sec, f"Grup-{g_id} formasyon seçimi"))
        k_for = f'filo.formasyon("{isim}", aralik={aralik}, is_3d={is_3d})'
        komutlar.append((k_for, f"{isim} | aralık={aralik}m | Grup-{g_id}"))

        if self.chk_takip.isChecked():
            k_mod = f"filo.change_mode(g_id={g_id}, new_mode=1)"
            komutlar.append((k_mod, "Lider takibi aktif edildi"))

        for k, a in komutlar:
            komut_gonder(k, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, seviye_tespit(s)))
            self.komut_uretildi.emit(k, a)
