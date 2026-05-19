"""
ROV Paneli — Mevcut ROV'ları listeler, lider seçimine izin verir.
"""
from __future__ import annotations

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QListWidget, QListWidgetItem,
    QGroupBox, QLabel, QPushButton, QAbstractItemView,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont

from UI.tema import (
    VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI, TURUNCU,
    PANEL, PANEL_KENAR, GOREV_RENK,
)
from UI.kopru import komut_gonder, sim_bagli_mi


_ROL_RENKLERi = {1: YESiL, 0: METiN_KOYU}
_ROL_METiN    = {1: "LİDER", 0: "TAKİPÇİ"}

# GAT kodu renk ve açıklama eşlemesi (0=OK, 1=Engel, 2=Çarpışma, 3=Kopma, 4=Kayıp, 5=Uzak)
_GAT_RENKLERi = {
    0: YESiL,   # OK
    1: SARI,    # Engel yakın
    2: KIRMIZI, # Çarpışma riski
    3: "#ff6600",# Bağlantı koptu
    4: "#cc00ff",# Konum kayıp
    5: TURUNCU, # Liderden uzak
}
_GAT_METiN = {
    0: "OK", 1: "ENGEL", 2: "ÇARPIŞMA", 3: "KOPMA", 4: "KAYIP", 5: "UZAK"
}


class ROVPanel(QWidget):
    rov_secildi       = pyqtSignal(int)      # Seçili ROV ID
    komut_uretildi    = pyqtSignal(str, str) # (komut_str, aciklama)
    lider_talep       = pyqtSignal(int)      # Lider yap talebi → surucu_panel
    takipci_talep     = pyqtSignal(int)      # Takipçi yap talebi → surucu_panel

    def __init__(self, sinyal=None, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        self._rovlar: list[dict] = []

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        kutu = QGroupBox("ROV LİSTESİ")
        kutu_lay = QVBoxLayout(kutu)
        kutu_lay.setSpacing(6)

        # Liste
        self.liste = QListWidget()
        self.liste.setSelectionMode(QAbstractItemView.SingleSelection)
        self.liste.itemSelectionChanged.connect(self._secim_degisti)
        kutu_lay.addWidget(self.liste)

        # Alt butonlar
        btn_lay = QHBoxLayout()
        btn_lay.setSpacing(4)

        self.btn_lider_yap = QPushButton("★ Lider Yap")
        self.btn_lider_yap.setToolTip("Seçili ROV'u lider olarak ata")
        self.btn_lider_yap.clicked.connect(self._lider_yap)
        self.btn_lider_yap.setEnabled(False)

        self.btn_takipci_yap = QPushButton("Takipçi Yap")
        self.btn_takipci_yap.setToolTip("Seçili ROV'u takipçi olarak ata")
        self.btn_takipci_yap.clicked.connect(self._takipci_yap)
        self.btn_takipci_yap.setEnabled(False)

        btn_lay.addWidget(self.btn_lider_yap)
        btn_lay.addWidget(self.btn_takipci_yap)
        kutu_lay.addLayout(btn_lay)

        # Seçili ROV bilgisi
        self.lbl_bilgi = QLabel("— ROV seçilmedi —")
        self.lbl_bilgi.setAlignment(Qt.AlignCenter)
        self.lbl_bilgi.setStyleSheet(f"color: {METiN_KOYU}; font-size: 8pt; padding: 2px;")
        kutu_lay.addWidget(self.lbl_bilgi)

        lay.addWidget(kutu)

    # ── Güncelleme ────────────────────────────────────────────────────────────
    def rov_listesini_guncelle(self, rovlar: list[dict]):
        self._rovlar = rovlar
        secili_id = self._secili_id()

        self.liste.blockSignals(True)
        self.liste.clear()
        for rov in rovlar:
            rid      = rov["id"]
            rol      = rov.get("rol", 0)
            gorev    = rov.get("gorev", "idle")
            gps      = rov.get("gps", (0, 0, 0))
            gat      = rov.get("gat_kodu", 0)
            batarya  = rov.get("batarya", 1.0)
            hiz      = rov.get("hiz", 0.0)

            rol_txt   = _ROL_METiN.get(rol, "?")
            gat_txt   = _GAT_METiN.get(gat, f"GAT:{gat}")
            gat_renk  = _GAT_RENKLERi.get(gat, METiN_KOYU)
            # Ön plan rengi: lider=yeşil, diğerleri GAT koduna göre
            on_renk   = YESiL if rol == 1 else gat_renk

            bat_yuzde = int(min(max(batarya * 100, 0), 100)) if batarya <= 1.0 else int(min(batarya, 100))
            bat_bar   = "█" * (bat_yuzde // 10) + "░" * (10 - bat_yuzde // 10)

            metin = (
                f"  ROV-{rid}  [{rol_txt}]  {gorev.upper()}  ● {gat_txt}\n"
                f"  X:{gps[0]:+.1f}  Y:{gps[1]:+.1f}  Z:{gps[2]:+.1f}"
                f"   {bat_bar} {bat_yuzde}%   {hiz:.1f}m/s"
            )

            item = QListWidgetItem(metin)
            item.setData(Qt.UserRole, rid)
            item.setForeground(QColor(on_renk))
            item.setFont(QFont("Consolas", 8))

            # Lider → özel arka plan; kritik GAT → kırmızımsı arka plan
            if rol == 1:
                item.setBackground(QColor("#001a0d"))
            elif gat in (2, 3):
                item.setBackground(QColor("#1a0000"))
            self.liste.addItem(item)

            # Önceki seçimi koru
            if rid == secili_id:
                self.liste.setCurrentItem(item)

        self.liste.blockSignals(False)

    def _secili_id(self) -> int | None:
        item = self.liste.currentItem()
        return item.data(Qt.UserRole) if item else None

    # ── Sinyal ────────────────────────────────────────────────────────────────
    def _secim_degisti(self):
        rid = self._secili_id()
        aktif = rid is not None
        self.btn_lider_yap.setEnabled(aktif)
        self.btn_takipci_yap.setEnabled(aktif)

        if rid is not None:
            rov = next((r for r in self._rovlar if r["id"] == rid), None)
            if rov:
                gps = rov.get("gps", (0, 0, 0))
                self.lbl_bilgi.setText(
                    f"ROV-{rid}  |  {_ROL_METiN.get(rov.get('rol',0), '?')}  "
                    f"|  {rov.get('gorev','idle').upper()}"
                )
            self.rov_secildi.emit(rid)
        else:
            self.lbl_bilgi.setText("— ROV seçilmedi —")

    # ── Buton işlemleri ───────────────────────────────────────────────────────
    def _lider_yap(self):
        rid = self._secili_id()
        if rid is None:
            return
        # Delegate: surucu_panel._lider_olustur(rid) → doğru group_id + cache mantığı orada
        self.lider_talep.emit(rid)

    def _takipci_yap(self):
        rid = self._secili_id()
        if rid is None:
            return
        # Delegate: surucu_panel._us_a_birak(rid) → grubundan çıkar, üsse al
        # Kullanıcı ardından SÜRÜ sekmesinden gruba sürükler
        self.takipci_talep.emit(rid)

    # ── Dışarıdan erişim ──────────────────────────────────────────────────────
    def secili_rov_id(self) -> int | None:
        return self._secili_id()
