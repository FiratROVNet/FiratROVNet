"""
Grup Paneli — Grupları gösterir, ROV ataması ve mod kontrolü sağlar.
"""
from __future__ import annotations

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QSpinBox, QPushButton, QListWidget, QListWidgetItem,
    QAbstractItemView, QComboBox,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont

from UI.tema import VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI, PANEL_KENAR
from UI.kopru import komut_gonder


class GrupPanel(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal=None, rov_panel_ref=None, parent=None):
        super().__init__(parent)
        self.sinyal        = sinyal
        self.rov_panel_ref = rov_panel_ref
        self._gruplar: dict[int, list[int]] = {}

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        kutu = QGroupBox("GRUP YÖNETİMİ")
        kutu_lay = QVBoxLayout(kutu)
        kutu_lay.setSpacing(8)

        # ── Grup listesi ────────────────────────────────────────────────────
        self.liste = QListWidget()
        self.liste.setMaximumHeight(110)
        kutu_lay.addWidget(self.liste)

        # ── Mod kontrolü ────────────────────────────────────────────────────
        mod_lay = QHBoxLayout()
        mod_lay.setSpacing(4)
        lbl_grup_id = QLabel("Grup:")
        lbl_grup_id.setStyleSheet(f"color: {METiN_KOYU};")
        lbl_grup_id.setFixedWidth(36)

        self.spin_grup_id = QSpinBox()
        self.spin_grup_id.setRange(0, 9)
        self.spin_grup_id.setFixedWidth(52)
        self.spin_grup_id.setToolTip("Hedef grup ID")

        self.cmb_mod = QComboBox()
        self.cmb_mod.addItem("Mod 1 — Lider Takibi", 1)
        self.cmb_mod.addItem("Mod 0 — Bağımsız (APF)", 0)
        self.cmb_mod.setToolTip("Mod 1: Takipçiler liderle hareket eder\nMod 0: Takipçiler bağımsız")

        btn_mod_uygula = QPushButton("Uygula")
        btn_mod_uygula.setFixedWidth(70)
        btn_mod_uygula.clicked.connect(self._mod_uygula)

        mod_lay.addWidget(lbl_grup_id)
        mod_lay.addWidget(self.spin_grup_id)
        mod_lay.addWidget(self.cmb_mod, 1)
        mod_lay.addWidget(btn_mod_uygula)
        kutu_lay.addLayout(mod_lay)

        # ── Otomatik formasyon seçimi ───────────────────────────────────────
        btn_oto_form = QPushButton("⬡  Otomatik Formasyon Seç")
        btn_oto_form.setObjectName("btn_formasyon_sec")
        btn_oto_form.setToolTip("Convex hull bazlı en uygun formasyon seçilir")
        btn_oto_form.clicked.connect(self._oto_formasyon)
        kutu_lay.addWidget(btn_oto_form)

        lay.addWidget(kutu)

    # ── Güncelleme ────────────────────────────────────────────────────────────
    def grup_listesini_guncelle(self, gruplar: dict[int, list[int]]):
        self._gruplar = gruplar
        self.liste.clear()
        for g_id, rovlar in sorted(gruplar.items()):
            item = QListWidgetItem(f"  Grup-{g_id}   →   ROV'lar: {rovlar}")
            item.setData(Qt.UserRole, g_id)
            item.setFont(QFont("Consolas", 8))
            item.setForeground(QColor(VURGU))
            self.liste.addItem(item)

    def rov_secim_guncelle(self, rov_id: int):
        """ROV panelinden gelen seçim: ilgili grubu otomatik seç."""
        for g_id, rovlar in self._gruplar.items():
            if rov_id in rovlar:
                self.spin_grup_id.setValue(g_id)
                break

    # ── Buton işlemleri ───────────────────────────────────────────────────────
    def _mod_uygula(self):
        g_id = self.spin_grup_id.value()
        mod  = self.cmb_mod.currentData()
        komut = f"filo.change_mode(g_id={g_id}, new_mode={mod})"
        aciklama = (
            f"Grup-{g_id} → {'Lider takip modu' if mod == 1 else 'Bağımsız mod'}"
        )
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)

    def _oto_formasyon(self):
        g_id  = self.spin_grup_id.value()
        komut = f"filo.formasyon_sec(g_id={g_id})"
        aciklama = f"Grup-{g_id} → Otomatik formasyon seçildi"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)

    def secili_grup_id(self) -> int:
        return self.spin_grup_id.value()
