"""
Sürü Yönetim Paneli — Sürükle-Bırak ROV Grubu Organizasyonu

ROV kartları (80px):  ID · GAT rengi · GAT durum · Batarya% · Hız · Görev
Lider Grubu altında:
  • 📊 İstatistik çubuğu (ROV sayısı, ort. batarya, ort. hız)
  • 🔇 Mod seçici (Bağımsız / Lider Takip)
  • 🎯 Hedef konum → tüm gruba git() komutu
  • ⬡ Formasyon hızlı butonlar (V · Çizgi · Üçgen · Daire · Ok · Elmas)
  • 📋 Görev ata (Alan Tarama / Arama Kurtarma / İmha)
"""
from __future__ import annotations
import time as _time

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QScrollArea,
    QFrame, QLabel, QPushButton, QSizePolicy, QMenu, QAction,
    QComboBox, QLineEdit, QDialog, QFormLayout, QDialogButtonBox, QCheckBox,
)
from PyQt5.QtCore import Qt, pyqtSignal, QMimeData, QPoint
from PyQt5.QtGui import QDrag, QFont, QDoubleValidator, QIntValidator

from UI.tema import (
    VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI, TURUNCU,
    PANEL, PANEL_KENAR, BUTON_NORMAL,
)
from UI.kopru import komut_gonder, grup_id_oku, ilk_bos_grup_id
from UI.minimap_secim import haritadan_sec_butonu

_MIME = "application/x-rov-id"

_GAT_RENK  = {0: YESiL, 1: SARI, 2: KIRMIZI, 3: "#ff6600", 4: "#cc00ff", 5: TURUNCU}
_GAT_METIN = {0: "OK", 1: "ENGEL", 2: "ÇARP", 3: "KOPMA", 4: "KAYIP", 5: "UZAK"}

_FORMASYONLAR = [
    ("V",  "V_SHAPE"),
    ("─",  "LINE"),
    ("△",  "TRIANGLE"),
    ("○",  "CIRCLE"),
    ("→",  "ARROW"),
    ("◇",  "DIAMOND"),
]

_GOREVLER = [
    ("Alan Tarama",      "alan_tarama"),
    ("Arama & Kurtarma", "arama_kurtarma"),
    ("İmha",             "imha"),
]

_BTN_BASE = f"""
    QPushButton {{
        background: {BUTON_NORMAL};
        color: {METiN};
        border: 1px solid {PANEL_KENAR};
        border-radius: 4px;
        padding: 2px 6px;
        font-family: Consolas;
        font-size: 9pt;
    }}
    QPushButton:hover {{ background: #253040; border-color: {VURGU}; }}
    QPushButton:pressed {{ background: #0a1520; }}
"""

_INPUT_CSS = f"""
    QLineEdit {{
        background: #0c111a;
        color: {METiN};
        border: 1px solid {PANEL_KENAR};
        border-radius: 3px;
        padding: 1px 4px;
        font-family: Consolas;
        font-size: 8pt;
    }}
    QLineEdit:focus {{ border-color: {VURGU}; }}
"""

_COMBO_CSS = f"""
    QComboBox {{
        background: {BUTON_NORMAL};
        color: {METiN};
        border: 1px solid {PANEL_KENAR};
        border-radius: 3px;
        padding: 2px 6px;
        font-family: Consolas;
        font-size: 8pt;
    }}
    QComboBox::drop-down {{ border: none; }}
    QComboBox QAbstractItemView {{
        background: #0d1520;
        color: {METiN};
        selection-background-color: #1a2535;
    }}
"""


# ─────────────────────────────────────────────────────────────────────────────
class ROVKarti(QFrame):
    """Sürüklenebilir ROV durum kartı — 80px yükseklik."""

    def __init__(self, rov_id: int, veri: dict, draggable: bool = True, parent=None):
        super().__init__(parent)
        self.rov_id    = rov_id
        self.draggable = draggable
        self._drag_pos: QPoint | None = None

        self.setFixedHeight(80)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        if draggable:
            self.setCursor(Qt.OpenHandCursor)

        lay = QHBoxLayout(self)
        lay.setContentsMargins(6, 4, 6, 4)
        lay.setSpacing(6)

        self._serit = QFrame()
        self._serit.setFixedWidth(5)
        lay.addWidget(self._serit)

        info = QVBoxLayout()
        info.setSpacing(2)

        self._lbl_id    = QLabel(f"ROV-{rov_id}")
        self._lbl_id.setFont(QFont("Consolas", 9, QFont.Bold))

        self._lbl_durum = QLabel()
        self._lbl_durum.setFont(QFont("Consolas", 7))

        self._lbl_gps   = QLabel()
        self._lbl_gps.setFont(QFont("Consolas", 7))

        self._lbl_gorev = QLabel()
        self._lbl_gorev.setFont(QFont("Consolas", 7))

        info.addWidget(self._lbl_id)
        info.addWidget(self._lbl_durum)
        info.addWidget(self._lbl_gps)
        info.addWidget(self._lbl_gorev)
        lay.addLayout(info, 1)

        if draggable:
            drag_ico = QLabel("⠿")
            drag_ico.setStyleSheet(f"color:{METiN_KOYU}; font-size:14pt;")
            lay.addWidget(drag_ico)

        self.guncelle(veri)

    def guncelle(self, v: dict):
        gat   = v.get("gat_kodu", 0)
        renk  = _GAT_RENK.get(gat, YESiL)
        bat   = v.get("batarya", 1.0)
        pct   = int(min(max(bat * 100 if bat <= 1.0 else bat, 0), 100))
        hiz   = v.get("hiz", 0.0)
        gps   = v.get("gps", (0, 0, 0))
        gorev = v.get("gorev", "idle") or "idle"

        dolu  = "█" * (pct // 10)
        bos   = "░" * (10 - pct // 10)

        self._serit.setStyleSheet(f"background:{renk}; border-radius:2px;")
        self._lbl_id.setStyleSheet(f"color:{METiN};")
        self._lbl_durum.setText(f"● {_GAT_METIN.get(gat,'?')}  🔋{dolu}{bos} {pct}%  {hiz:.1f}m/s")
        self._lbl_durum.setStyleSheet(f"color:{renk};")
        g_id = grup_id_oku(v)
        g_txt = f"  ▸G-{g_id}" if g_id is not None else "  ▸üs"
        self._lbl_id.setText(f"ROV-{self.rov_id}{g_txt}")
        self._lbl_gps.setText(f"X:{gps[0]:+.0f}  Y:{gps[1]:+.0f}  Z:{gps[2]:+.0f}")
        self._lbl_gps.setStyleSheet(f"color:{METiN_KOYU};")
        self._lbl_gorev.setText(f"📋 {gorev}")
        self._lbl_gorev.setStyleSheet(f"color:{METiN_KOYU};")
        self.setStyleSheet(f"""
            ROVKarti {{
                background:{PANEL};
                border:1px solid {PANEL_KENAR};
                border-left:5px solid {renk};
                border-radius:5px;
            }}
            ROVKarti:hover {{ border-color:{VURGU}; border-left-color:{renk}; }}
        """)

    def mousePressEvent(self, e):
        if self.draggable and e.button() == Qt.LeftButton:
            self._drag_pos = e.pos()
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if not (self.draggable and self._drag_pos and e.buttons() & Qt.LeftButton):
            return
        if (e.pos() - self._drag_pos).manhattanLength() < 10:
            return
        drag = QDrag(self)
        mime = QMimeData()
        mime.setData(_MIME, str(self.rov_id).encode())
        drag.setMimeData(mime)
        drag.setPixmap(self.grab())
        drag.setHotSpot(e.pos())
        self._drag_pos = None
        drag.exec_(Qt.MoveAction)

    def mouseReleaseEvent(self, e):
        self._drag_pos = None
        super().mouseReleaseEvent(e)


# ─────────────────────────────────────────────────────────────────────────────
class _DropAlan(QFrame):
    rov_birakildi = pyqtSignal(int)

    _NORMAL_CSS = ""
    _HOVER_CSS  = ""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)

    def dragEnterEvent(self, e):
        if e.mimeData().hasFormat(_MIME):
            e.acceptProposedAction()
            self.setStyleSheet(self._HOVER_CSS)

    def dragMoveEvent(self, e):
        if e.mimeData().hasFormat(_MIME):
            e.acceptProposedAction()

    def dragLeaveEvent(self, e):
        self.setStyleSheet(self._NORMAL_CSS)

    def dropEvent(self, e):
        if e.mimeData().hasFormat(_MIME):
            rid = int(bytes(e.mimeData().data(_MIME)).decode())
            e.acceptProposedAction()
            self.setStyleSheet(self._NORMAL_CSS)
            self.rov_birakildi.emit(rid)


# ─────────────────────────────────────────────────────────────────────────────
class BeklemeBolgesi(_DropAlan):
    menu_istendi = pyqtSignal(int, QPoint)  # (rov_id, global_pos)

    _NORMAL_CSS = f"""
        BeklemeBolgesi {{
            background:#0a0e14;
            border:2px dashed {SARI};
            border-radius:8px;
        }}
    """
    _HOVER_CSS = f"""
        BeklemeBolgesi {{
            background:#1a1500;
            border:2px solid #ffee00;
            border-radius:8px;
        }}
    """

    def __init__(self, sinyal=None, parent=None):
        super().__init__(parent)
        self.sinyal   = sinyal
        self._kartlar: dict[int, ROVKarti] = {}

        self.setMinimumWidth(240)
        self.setMaximumWidth(255)
        self.setStyleSheet(self._NORMAL_CSS)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(6)

        self._lbl_baslik = QLabel("ÜS BÖLGESİ  (0)")
        self._lbl_baslik.setFont(QFont("Consolas", 9, QFont.Bold))
        self._lbl_baslik.setStyleSheet(f"color:{SARI}; border:none;")
        self._lbl_baslik.setAlignment(Qt.AlignCenter)
        lay.addWidget(self._lbl_baslik)

        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"border:1px solid {SARI};")
        lay.addWidget(sep)

        hint = QLabel("↓ ROV’ları buraya bırak\nSağ tık → İşlem menüsü")
        hint.setFont(QFont("Consolas", 7))
        hint.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        hint.setAlignment(Qt.AlignCenter)
        lay.addWidget(hint)

        self._liste_lay = QVBoxLayout()
        self._liste_lay.setSpacing(4)
        self._liste_lay.addStretch()
        lay.addLayout(self._liste_lay, 1)

    def rov_ekle(self, rid: int, veri: dict):
        if rid in self._kartlar:
            self._kartlar[rid].guncelle(veri)
            return
        kart = ROVKarti(rid, veri, draggable=True)
        kart.setContextMenuPolicy(Qt.CustomContextMenu)
        kart.customContextMenuRequested.connect(
            lambda pos, r=rid, k=kart: self.menu_istendi.emit(r, k.mapToGlobal(pos))
        )
        self._kartlar[rid] = kart
        self._liste_lay.insertWidget(self._liste_lay.count() - 1, kart)
        self._baslik_guncelle()

    def rov_cikar(self, rid: int):
        if rid not in self._kartlar:
            return
        kart = self._kartlar.pop(rid)
        self._liste_lay.removeWidget(kart)
        kart.deleteLater()
        self._baslik_guncelle()

    def veri_guncelle(self, rid: int, veri: dict):
        if rid in self._kartlar:
            self._kartlar[rid].guncelle(veri)

    def rov_idleri(self) -> list[int]:
        return list(self._kartlar.keys())

    def _baslik_guncelle(self):
        self._lbl_baslik.setText(f"ÜS BÖLGESİ  ({len(self._kartlar)})")


# ─────────────────────────────────────────────────────────────────────────────
class LiderGrubu(_DropAlan):
    kaldir_istendi     = pyqtSignal(int)
    takipci_us_istendi = pyqtSignal(int)
    menu_istendi       = pyqtSignal(int, QPoint)  # (rov_id, global_pos)
    komut_uretildi     = pyqtSignal(str, str)

    _NORMAL_CSS = f"""
        LiderGrubu {{
            background:#0d1520;
            border:2px solid {VURGU};
            border-radius:8px;
        }}
    """
    _HOVER_CSS = f"""
        LiderGrubu {{
            background:#001a30;
            border:2px solid #80eaff;
            border-radius:8px;
        }}
    """

    def __init__(self, lider_id: int, lider_veri: dict, g_idx: int, sinyal=None, parent=None):
        super().__init__(parent)
        self.lider_id = lider_id
        self.g_idx    = g_idx
        self.sinyal   = sinyal
        self._kartlar: dict[int, ROVKarti] = {}
        self._mod     = 0  # 0=Bağımsız, 1=Lider Takip

        self.setMinimumWidth(285)
        self.setMaximumWidth(300)
        self.setStyleSheet(self._NORMAL_CSS)
        self.setContextMenuPolicy(Qt.CustomContextMenu)
        self.customContextMenuRequested.connect(self._mod_menu)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(5)

        # ── Başlık ──
        hdr = QHBoxLayout()
        lbl_hdr = QLabel(f"GRUP-{g_idx}  ·  LİDER ROV-{lider_id}")
        lbl_hdr.setFont(QFont("Consolas", 8, QFont.Bold))
        lbl_hdr.setStyleSheet(f"color:{VURGU}; border:none;")
        hdr.addWidget(lbl_hdr, 1)
        self._mod_lbl = QLabel("⚙ Bağımsız")
        self._mod_lbl.setFont(QFont("Consolas", 7))
        self._mod_lbl.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        self._mod_lbl.setToolTip("Sağ tık ile modu değiştir")
        hdr.addWidget(self._mod_lbl)
        btn_x = QPushButton("✕")
        btn_x.setFixedSize(22, 22)
        btn_x.setToolTip("Liderliği kaldır — tüm ROV'lar üsse döner")
        btn_x.setStyleSheet(f"color:{KIRMIZI};font-weight:bold;border:none;background:transparent;")
        btn_x.clicked.connect(lambda: self.kaldir_istendi.emit(self.lider_id))
        hdr.addWidget(btn_x)
        lay.addLayout(hdr)

        # ── Lider kartı ──
        self._lider_kart = ROVKarti(lider_id, lider_veri, draggable=False)
        lay.addWidget(self._lider_kart)

        # ── Takipçi alanı ──
        self._lbl_tak = QLabel("TAKİPÇİLER  (0)  ↓ sürükle")
        self._lbl_tak.setFont(QFont("Consolas", 7))
        self._lbl_tak.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(self._lbl_tak)

        self._liste_lay = QVBoxLayout()
        self._liste_lay.setSpacing(4)
        self._liste_lay.addStretch()
        lay.addLayout(self._liste_lay, 1)

        # ── Ayırıcı ──
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet(f"border:1px solid {PANEL_KENAR};")
        lay.addWidget(sep)

        # ── 📊 İstatistik ──
        self._lbl_stat = QLabel("📊  0 ROV  ·  🔋 –%  ·  – m/s")
        self._lbl_stat.setFont(QFont("Consolas", 7))
        self._lbl_stat.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(self._lbl_stat)

        # ── 🎯 Hedef konum ──
        hedef_lbl = QLabel("🎯 Hedef:")
        hedef_lbl.setFont(QFont("Consolas", 7))
        hedef_lbl.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(hedef_lbl)

        coord_lay = QHBoxLayout()
        coord_lay.setSpacing(3)
        dbl = QDoubleValidator(-9999.0, 9999.0, 1)

        def _inp(tip: str) -> QLineEdit:
            e = QLineEdit()
            e.setPlaceholderText(tip)
            e.setValidator(dbl)
            e.setStyleSheet(_INPUT_CSS)
            e.setFixedHeight(22)
            return e

        self._ex = _inp("X")
        self._ey = _inp("Y")
        self._ez = _inp("Z")
        for w in (self._ex, self._ey, self._ez):
            coord_lay.addWidget(w, 1)

        btn_git = QPushButton("→ Git")
        btn_git.setFixedHeight(22)
        btn_git.setStyleSheet(_BTN_BASE)
        btn_git.clicked.connect(self._git_gonder)
        coord_lay.addWidget(btn_git)
        lay.addLayout(coord_lay)

        # ── ⬡ Formasyon hızlı seçici ──
        form_lbl = QLabel("⬡ Formasyon:")
        form_lbl.setFont(QFont("Consolas", 7))
        form_lbl.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(form_lbl)

        form_lay = QHBoxLayout()
        form_lay.setSpacing(3)
        for sembol, isim in _FORMASYONLAR:
            btn = QPushButton(sembol)
            btn.setFixedSize(36, 26)
            btn.setToolTip(isim)
            btn.setStyleSheet(_BTN_BASE)
            btn.clicked.connect(lambda _, n=isim: self._formasyon_uygula(n))
            form_lay.addWidget(btn)
        lay.addLayout(form_lay)

        # ── 📋 Görev ata ──
        gorev_lbl = QLabel("📋 Görev:")
        gorev_lbl.setFont(QFont("Consolas", 7))
        gorev_lbl.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(gorev_lbl)

        gorev_ust = QHBoxLayout()
        gorev_ust.setSpacing(3)
        self._cmb_gorev = QComboBox()
        for ad, _ in _GOREVLER:
            self._cmb_gorev.addItem(ad)
        self._cmb_gorev.setStyleSheet(_COMBO_CSS)
        self._cmb_gorev.setFixedHeight(24)
        btn_bas = QPushButton("▶ Başlat")
        btn_bas.setFixedHeight(24)
        btn_bas.setStyleSheet(
            _BTN_BASE.replace(f"color: {METiN}", f"color: {YESiL}")
        )
        btn_bas.clicked.connect(self._gorev_baslat)
        btn_dur = QPushButton("■ Durdur")
        btn_dur.setFixedHeight(24)
        btn_dur.setStyleSheet(
            _BTN_BASE.replace(f"color: {METiN}", f"color: {KIRMIZI}")
        )
        btn_dur.clicked.connect(self._gorev_durdur)
        gorev_ust.addWidget(self._cmb_gorev, 1)
        gorev_ust.addWidget(btn_bas)
        gorev_ust.addWidget(btn_dur)
        lay.addLayout(gorev_ust)

        # Alan koordinatları (X1,Y1 → X2,Y2)
        alan_lbl = QLabel("  Alan: X1  Y1  →  X2  Y2")
        alan_lbl.setFont(QFont("Consolas", 7))
        alan_lbl.setStyleSheet(f"color:{METiN_KOYU}; border:none;")
        lay.addWidget(alan_lbl)

        alan_lay = QHBoxLayout()
        alan_lay.setSpacing(3)
        self._ax1 = _inp("X1")
        self._ay1 = _inp("Y1")
        self._ax2 = _inp("X2")
        self._ay2 = _inp("Y2")
        for w in (self._ax1, self._ay1, self._ax2, self._ay2):
            alan_lay.addWidget(w, 1)
        lay.addLayout(alan_lay)

        def _alan_yaz(x1, y1, x2, y2):
            for w, val in (
                (self._ax1, x1), (self._ay1, y1), (self._ax2, x2), (self._ay2, y2),
            ):
                w.setText(f"{val:.1f}")
                w.setStyleSheet(_INPUT_CSS)

        harita_lay = QHBoxLayout()
        harita_lay.addStretch()
        harita_lay.addWidget(
            haritadan_sec_butonu("alan", _alan_yaz, sinyal=self.sinyal)
        )
        lay.addLayout(harita_lay)

    # ── Takipçi yönetimi ──────────────────────────────────────────────────
    def takipci_ekle(self, rid: int, veri: dict):
        if rid == self.lider_id or rid in self._kartlar:
            if rid in self._kartlar:
                self._kartlar[rid].guncelle(veri)
            return
        kart = ROVKarti(rid, veri, draggable=True)
        kart.setContextMenuPolicy(Qt.CustomContextMenu)
        kart.customContextMenuRequested.connect(
            lambda pos, r=rid, k=kart: self.menu_istendi.emit(r, k.mapToGlobal(pos))
        )
        self._kartlar[rid] = kart
        self._liste_lay.insertWidget(self._liste_lay.count() - 1, kart)
        self._baslik_guncelle()

    def takipci_cikar(self, rid: int):
        if rid not in self._kartlar:
            return
        kart = self._kartlar.pop(rid)
        self._liste_lay.removeWidget(kart)
        kart.deleteLater()
        self._baslik_guncelle()

    def veri_guncelle(self, rid: int, veri: dict):
        if rid == self.lider_id:
            self._lider_kart.guncelle(veri)
        elif rid in self._kartlar:
            self._kartlar[rid].guncelle(veri)

    def takipci_idleri(self) -> list[int]:
        return list(self._kartlar.keys())

    def _baslik_guncelle(self):
        self._lbl_tak.setText(f"TAKİPÇİLER  ({len(self._kartlar)})")

    def stat_guncelle_veri(self, veri_map: dict[int, dict]):
        tum_ids = [self.lider_id] + list(self._kartlar.keys())
        batlar  = [veri_map[i]["batarya"] for i in tum_ids if i in veri_map]
        hizlar  = [veri_map[i]["hiz"]     for i in tum_ids if i in veri_map]
        n       = len(tum_ids)
        bat_ort = (sum(batlar) / len(batlar)) * 100 if batlar else 0
        hiz_ort = sum(hizlar) / len(hizlar) if hizlar else 0
        self._lbl_stat.setText(
            f"📊  {n} ROV  ·  🔋 {bat_ort:.0f}%  ·  {hiz_ort:.1f} m/s"
        )

    def _menu(self, rid: int, pos: QPoint):
        # Delegate to parent SurucuPanel via signal
        self.menu_istendi.emit(rid, pos)

    # ── Aksiyon komutları ─────────────────────────────────────────────────
    def _mod_menu(self, pos: QPoint):
        m = QMenu(self)
        m.setStyleSheet(f"""
            QMenu {{ background:#0d1520; color:{METiN}; border:1px solid {PANEL_KENAR}; }}
            QMenu::item:selected {{ background:#1a2535; }}
        """)
        lbl = m.addAction(f"Grup-{self.g_idx} Modunu Seç")
        lbl.setEnabled(False)
        m.addSeparator()
        a0 = QAction("■  Bağımsız" + ("  ✔" if self._mod == 0 else ""), m)
        a0.triggered.connect(lambda: self._mod_uygula(0))
        a1 = QAction("▶  Lider Takip" + ("  ✔" if self._mod == 1 else ""), m)
        a1.triggered.connect(lambda: self._mod_uygula(1))
        m.addAction(a0)
        m.addAction(a1)
        m.exec_(self.mapToGlobal(pos))

    def _mod_uygula(self, idx: int):
        self._mod = idx
        self._mod_lbl.setText("⚙ " + ("Lider Takip" if idx == 1 else "Bağımsız"))
        k = f"filo.change_mode(g_id={self.g_idx}, new_mode={idx})"
        komut_gonder(k)
        self.komut_uretildi.emit(k, f"Grup-{self.g_idx} → Mod-{idx}")

    def _git_gonder(self):
        x_s = self._ex.text().strip().replace(",", ".")
        y_s = self._ey.text().strip().replace(",", ".")
        z_s = self._ez.text().strip().replace(",", ".")
        if not x_s or not y_s:
            return
        x = float(x_s)
        y = float(y_s)
        z = float(z_s) if z_s else 0.0
        k0 = f"filo.git(rov_id={self.lider_id}, x={x}, y={y}, z={z}, ai=True)"
        komut_gonder(k0)
        if self._kartlar:
            k1 = (
                f"[filo.git(rov_id=r, x={x}, y={y}, z={z}, ai=True) "
                f"for r in {list(self._kartlar.keys())}]"
            )
            komut_gonder(k1)
            self.komut_uretildi.emit(k1, f"Grup-{self.g_idx} → ({x},{y},{z})")
        else:
            self.komut_uretildi.emit(k0, f"ROV-{self.lider_id} → ({x},{y},{z})")

    def _formasyon_uygula(self, isim: str):
        g_id = self.g_idx
        # Mevcut aralığı koru — yoksa varsayılan 20 kullan
        k = (
            f"_af=getattr(filo,'aktif_formasyon',None);"
            f"isinstance(_af,dict) or setattr(filo,'aktif_formasyon',{{}});"
            f"_af=filo.aktif_formasyon;"
            f"_aralik=(_af.get({g_id}) or {{}}).get('aralik',20);"
            f"_af[{g_id}]={{'id':'{isim}','aralik':_aralik,'is_3d':False,'yaw':0,'g_id':{g_id}}};"
            f"filo.change_mode(g_id={g_id}, new_mode=1)"
        )
        komut_gonder(k)
        # UI mod etiketini güncelle (backend ile senkronize et)
        self._mod = 1
        self._mod_lbl.setText("⚙ Lider Takip")
        self.komut_uretildi.emit(k, f"Grup-{g_id} → Formasyon: {isim} | mod=1 (Lider Takip)")

    def _alan_oku(self) -> tuple[float, float, float, float] | None:
        """X1,Y1,X2,Y2 alanlarını oku. Eksikse None döner."""
        def _f(w: QLineEdit) -> float | None:
            t = w.text().strip().replace(",", ".")
            try:
                return float(t) if t else None
            except ValueError:
                return None
        vals = [_f(w) for w in (self._ax1, self._ay1, self._ax2, self._ay2)]
        if any(v is None for v in vals):
            return None
        return tuple(vals)  # type: ignore[return-value]

    def _gorev_baslat(self):
        idx   = self._cmb_gorev.currentIndex()
        _, tp = _GOREVLER[idx]
        g_id  = self.g_idx
        alan  = self._alan_oku()
        if alan is None:
            for w in (self._ax1, self._ay1, self._ax2, self._ay2):
                w.setStyleSheet(_INPUT_CSS + "QLineEdit { border-color: #ff1744; }")
            return
        for w in (self._ax1, self._ay1, self._ax2, self._ay2):
            w.setStyleSheet(_INPUT_CSS)
        x1, y1, x2, y2 = alan

        # ── Görev komutu ──────────────────────────────────────────────────────
        if tp == "alan_tarama":
            k_baslat = (f"filo.alan_tarama_gorevi.baslat("
                        f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), "
                        f"surekli_tarama=True, sessiz=True)")
        elif tp == "arama_kurtarma":
            k_baslat = (f"filo.arama_kurtarma_gorevi.baslat("
                        f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), "
                        f"min_confidence=0.45, sessiz=True)")
        elif tp == "imha":
            k_baslat = (f"filo.imha_gorevi.alan_imha_baslat("
                        f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), sessiz=True)")
        else:
            return

        # Tüm komutları TEK bir exec() çağrısında sırasal çalıştır.
        # Birden fazla ayrı komut_gonder() çağrısı ayrı thread'ler başlatır;
        # thread zamanlama belirsizliği nedeniyle durdur() baslat()'tan SONRA
        # çalışarak yeni planı hemen popping'leyip ROV'ları idle'a alabilir.
        komutlar = [
            f"filo.alan_tarama_gorevi.durdur(grup_id={g_id})",
            f"filo.arama_kurtarma_gorevi.durdur(lideri_takip_et=False)",
            f"filo.imha_gorevi.durdur(lideri_takip_et=False)",
            f"[filo._rov_hedefleri.pop(r.id, None) for r in (filo.g_rovs.get({g_id}) or []) if r]",
            k_baslat,
        ]
        komut_gonder("\n".join(komutlar))
        self.komut_uretildi.emit(
            k_baslat, f"Grup-{g_id} → Görev: {_GOREVLER[idx][0]} | Eski görevler durduruldu"
        )

    def _gorev_durdur_hepsi(self, sessizce: bool = False):
        """Bu grup için tüm görev tiplerini durdurur (navigasyon hedefleri korunur)."""
        g_id = self.g_idx
        for k in (
            f"filo.alan_tarama_gorevi.durdur(grup_id={g_id})",
            "filo.arama_kurtarma_gorevi.durdur(lideri_takip_et=False)",
            "filo.imha_gorevi.durdur(lideri_takip_et=False)",
            "ui_minimap_gorev_alan_temizle(app)",
        ):
            komut_gonder(k)
        if not sessizce:
            self.komut_uretildi.emit(
                f"# Grup-{g_id} tüm görevler durduruldu",
                f"Grup-{g_id} → Alan tarama / Arama / İmha durduruldu"
            )

    def _gorev_durdur(self):
        self._gorev_durdur_hepsi(sessizce=False)


# ─────────────────────────────────────────────────────────────────────────────
class _HintPaneli(QFrame):
    """Sağ boş alanda temaya uygun ipucu / dekorasyon paneli."""

    _SATIRLAR = [
        (VURGU,      "◈  SÜRÜ YÖNETİM PANELİ"),
        (METiN_KOYU, ""),
        (METiN_KOYU, "  ROV Grubu Oluşturmak İçin:"),
        (METiN,      "  1 · Üs bölgesinden ROV kartını"),
        (METiN,      "      sürükle → grup kutusuna bırak"),
        (METiN,      "  2 · Lider ROV çift tıkla → yeni grup"),
        (METiN,      "  3 · Sağ tık → işlem menüsü"),
        (METiN_KOYU, ""),
        (METiN_KOYU, "  Formasyon Kısayolları:"),
        (METiN,      f"  {'  '.join(s for s, _ in _FORMASYONLAR):<24}"),
        (METiN_KOYU, ""),
        (METiN_KOYU, "  Görev Başlatmak İçin:"),
        (METiN,      "  Alan koordinatlarını gir"),
        (METiN,      "  ▶ Başlat butonuna bas"),
        (METiN_KOYU, ""),
        (PANEL_KENAR, "─" * 32),
        (METiN_KOYU, "  Fırat-GNC  ·  Sürü Kontrol v2"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            _HintPaneli {{
                background: #0c1018;
                border: 1px solid {PANEL_KENAR};
                border-radius: 8px;
            }}
        """)
        self.setFixedWidth(220)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 14, 12, 14)
        lay.setSpacing(3)

        for renk, metin in self._SATIRLAR:
            lbl = QLabel(metin)
            lbl.setFont(QFont("Consolas", 8))
            lbl.setStyleSheet(f"color:{renk}; background:transparent; border:none;")
            lay.addWidget(lbl)

        lay.addStretch()

        # Alt dekor: küçük durum ikonu
        alt = QLabel("● SİSTEM AKTİF")
        alt.setFont(QFont("Consolas", 7))
        alt.setAlignment(Qt.AlignCenter)
        alt.setStyleSheet(f"color:{YESiL}; background:transparent; border:none;")
        lay.addWidget(alt)


# ─────────────────────────────────────────────────────────────────────────────
class SurucuPanel(QWidget):
    """Ana sürü yönetim paneli."""
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal=None, parent=None):
        super().__init__(parent)
        self.sinyal   = sinyal
        self._veri:          dict[int, dict]       = {}
        self._base:          set[int]              = set()
        self._liderler:      dict[int, LiderGrubu] = {}
        self._init_ok        = False
        self._baslangic_gps: dict[int, tuple]      = {}  # rov_id → başlangıç GPS
        self._son_sim_state: dict[int, tuple[int, int | None]] = {}  # {rid: (rol, grup_id)}
        # {rid: (deadline, beklenen (rol, grup_id))} — sim onayı gelene kadar UI geri alınmaz
        self._bekleyen_hareket: dict[int, tuple[float, tuple[int, int | None]]] = {}

        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        lay.setSpacing(6)

        # ── Üst stat çubuğu ──
        stat_lay = QHBoxLayout()
        self._stat = QLabel("Bağlantı bekleniyor...")
        self._stat.setFont(QFont("Consolas", 8))
        self._stat.setStyleSheet(f"color:{METiN_KOYU};")
        stat_lay.addWidget(self._stat, 1)

        btn_reset = QPushButton("◀  Tümünü Üsse Al")
        btn_reset.setFixedHeight(26)
        btn_reset.setStyleSheet(_BTN_BASE)
        btn_reset.clicked.connect(self._tum_use_al)
        stat_lay.addWidget(btn_reset)
        btn_rov_ekle = QPushButton("➕ ROV Ekle")
        btn_rov_ekle.setFixedHeight(26)
        btn_rov_ekle.setStyleSheet(
            _BTN_BASE.replace(f"background: {BUTON_NORMAL}", "background: #0d2a18")
            .replace(f"color: {METiN}", f"color: {YESiL}")
        )
        btn_rov_ekle.clicked.connect(self._sim_rov_ekle)
        stat_lay.addWidget(btn_rov_ekle)
        lay.addLayout(stat_lay)

        # ── Yatay scroll alanı ──
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self._scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self._scroll.setMinimumHeight(520)
        self._scroll.setStyleSheet(
            f"QScrollArea {{ background: #0a0d12; border: none; }}"
        )

        self._icerik = QWidget()
        self._icerik.setStyleSheet("background: #0a0d12;")
        self._h_lay  = QHBoxLayout(self._icerik)
        self._h_lay.setContentsMargins(4, 4, 4, 4)
        self._h_lay.setSpacing(12)
        self._h_lay.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self._us = BeklemeBolgesi(sinyal)
        self._us.rov_birakildi.connect(self._us_a_birak)
        self._us.menu_istendi.connect(self._menu_goster)
        self._h_lay.addWidget(self._us)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setFixedWidth(2)
        sep.setStyleSheet(f"background:{PANEL_KENAR};")
        self._h_lay.addWidget(sep)

        self._h_lay.addStretch()

        self._hint_paneli = _HintPaneli()
        self._h_lay.addWidget(self._hint_paneli, 0, Qt.AlignTop | Qt.AlignRight)

        self._scroll.setWidget(self._icerik)
        # viewport arka planını da zorla
        self._scroll.viewport().setStyleSheet("background: #0a0d12;")
        lay.addWidget(self._scroll, 1)

    def _grup_bul(self, g_idx: int) -> LiderGrubu | None:
        return next((g for g in self._liderler.values() if g.g_idx == g_idx), None)

    def _kullanilan_grup_idleri(self) -> set[int]:
        """UI ve sim verisindeki dolu grup numaraları (üs/None hariç)."""
        ids = {g.g_idx for g in self._liderler.values()}
        for r in self._veri.values():
            gid = grup_id_oku(r)
            if gid is not None:
                ids.add(gid)
        return ids

    def _sonraki_grup_id(self) -> int:
        return ilk_bos_grup_id(self._kullanilan_grup_idleri())

    def _bekleyen_kaydet(self, rid: int, beklenen: tuple[int, int | None], sure_s: float = 8.0):
        self._bekleyen_hareket[rid] = (_time.monotonic() + sure_s, beklenen)

    def _rov_konumu_bul(self, rid: int) -> tuple[str, int | None]:
        if rid in self._base:
            return "base", None
        if rid in self._liderler:
            return "leader", rid
        for lid, grup in self._liderler.items():
            if rid in grup.takipci_idleri():
                return "follower", lid
        return "unknown", None

    def _sim_statine_gore_yerlestir(self, rovlar: list[dict]):
        """Simülasyondaki group/role değişimlerini sürükle-bırak UI'ya yansıtır."""
        for rov in rovlar:
            rid = int(rov.get("id", -1))
            if rid < 0:
                continue

            pending = self._bekleyen_hareket.get(rid)
            if pending is not None and _time.monotonic() < pending[0]:
                continue

            hedef_gid = grup_id_oku(rov)
            rol = int(rov.get("rol", 0) or 0)
            konum, eski_lid = self._rov_konumu_bul(rid)
            hedef_grup = self._grup_bul(hedef_gid) if hedef_gid is not None else None

            # ── Zaten doğru konumdaysa yeniden yerleştirme ──────────────────
            if rol == 1 and hedef_gid is not None:
                lid_grup = self._liderler.get(rid)
                if lid_grup and lid_grup.g_idx == hedef_gid:
                    continue  # zaten doğru lider grubunda
            elif rol != 1 and hedef_gid is not None:
                if konum == "follower" and eski_lid is not None:
                    fg = self._liderler.get(eski_lid)
                    if fg and fg.g_idx == hedef_gid:
                        continue  # zaten doğru grupta takipçi
            else:  # grupsuz (üs)
                if konum == "base":
                    continue  # zaten üste
            # ───────────────────────────────────────────────────────────────

            # Her durumda önce eski konumdan çıkar
            if konum == "follower" and eski_lid is not None and eski_lid in self._liderler:
                self._liderler[eski_lid].takipci_cikar(rid)
            elif konum == "leader":
                grup = self._liderler.pop(rid, None)
                if grup:
                    g_idx = grup.g_idx
                    takipciler = list(grup.takipci_idleri())
                    self._h_lay.removeWidget(grup)
                    grup.deleteLater()
                    if takipciler:
                        yeni_lid = takipciler[0]
                        self._lider_olustur(yeni_lid, emit_komut=False, sim_g_id=g_idx)
                        ng = self._liderler.get(yeni_lid)
                        if ng:
                            for tid in takipciler[1:]:
                                ng.takipci_ekle(tid, self._veri.get(tid, {"id": tid}))
                elif hedef_gid is not None:
                    self._lider_kaldir(rid, emit_komut=False)
            elif konum == "base":
                self._base.discard(rid)
                self._us.rov_cikar(rid)

            # Şimdi yeni rolüne/grubuna yerleştir
            if rol == 1 and hedef_gid is not None:
                if hedef_grup is None:
                    self._lider_olustur(rid, emit_komut=False, sim_g_id=hedef_gid)
                elif hedef_grup.lider_id != rid:
                    self._lider_degistir(rid, hedef_grup.lider_id, hedef_gid, emit_komut=False)
            elif rol != 1 and hedef_gid is not None and hedef_grup is not None:
                hedef_grup.takipci_ekle(rid, self._veri.get(rid, rov))
            elif rol != 1 and hedef_gid is not None and hedef_grup is None:
                self._lider_olustur(rid, emit_komut=False, sim_g_id=hedef_gid)
            else:
                self._base.add(rid)
                self._us.rov_ekle(rid, self._veri.get(rid, rov))

    # ── Public API ────────────────────────────────────────────────────────
    def senkronize_et(self):
        """Sim state'ini zorla UI'ya yeniden yansıtır. Tüm kart/grup yapısını sıfırlayıp
        sim'deki rol/group_id değerlerine göre yeniden inşa eder."""
        # Mevcut tüm grup ve kartları temizle (emit_komut=False — sima komut gönderme)
        for lid in list(self._liderler.keys()):
            self._lider_kaldir(lid, emit_komut=False)
        self._base.clear()
        for i in range(self._us._liste_lay.count() - 1, -1, -1):
            item = self._us._liste_lay.itemAt(i)
            if item and item.widget():
                item.widget().deleteLater()
        # Durumu sıfırla ve mevcut veriyle yeniden başlat
        self._init_ok = False
        self._son_sim_state.clear()
        self._bekleyen_hareket.clear()
        if self._veri:
            self._ilk_yerles(list(self._veri.values()))
            self._init_ok = True

    def rov_listesini_guncelle(self, rovlar: list[dict]):
        self._veri = {r["id"]: r for r in rovlar}
        sim_ids = {r["id"] for r in rovlar}

        # İlk bağlantı: hiç ROV yokken açıldı, _init_ok False kalsın ta ki ROV gelene kadar
        if not self._init_ok:
            if rovlar:
                self._ilk_yerles(rovlar)
                self._init_ok = True
            return

        # Simülasyona yeni eklenen ROV'lar → grup/üs konumuna göre yerleştir
        bilinen_ids = (self._base
                       | set(self._liderler.keys())
                       | {rid for g in self._liderler.values() for rid in g.takipci_idleri()})
        for r in rovlar:
            rid = r["id"]
            if rid not in bilinen_ids:
                gps = r.get("gps", (0, 0, 0))
                self._baslangic_gps.setdefault(rid, tuple(gps))
                sim_rol = int(r.get("rol", 0) or 0)
                self._son_sim_state[rid] = (sim_rol, grup_id_oku(r))
                self._sim_statine_gore_yerlestir([r])

        # Simülasyondan çıkarılan ROV'lar → UI'dan temizle
        for rid in list(bilinen_ids - sim_ids):
            if rid in self._base:
                self._base.discard(rid)
                self._us.rov_cikar(rid)
            elif rid in self._liderler:
                self._lider_kaldir(rid, emit_komut=False)
            else:
                for g in list(self._liderler.values()):
                    if rid in g.takipci_idleri():
                        g.takipci_cikar(rid)
                        break

        # ── Sim-kaynaklı rol/grup değişikliklerini algıla → yeniden yerleştir ──
        # Süresi dolmuş token'ları önce temizle
        _now = _time.monotonic()
        self._bekleyen_hareket = {
            rid: kayit for rid, kayit in self._bekleyen_hareket.items() if _now < kayit[0]
        }
        for rov in rovlar:
            rid = rov["id"]
            sim_rol = int(rov.get("rol", 0) or 0)
            sim_gid = grup_id_oku(rov)
            yeni = (sim_rol, sim_gid)
            eski = self._son_sim_state.get(rid)
            pending = self._bekleyen_hareket.get(rid)
            if pending is not None:
                _, beklenen = pending
                if yeni == beklenen:
                    del self._bekleyen_hareket[rid]
                    self._son_sim_state[rid] = yeni
                    continue
                if _now < pending[0]:
                    continue
                del self._bekleyen_hareket[rid]
            self._son_sim_state[rid] = yeni
            if eski is None or eski == yeni:
                continue
            self._sim_statine_gore_yerlestir([rov])

        # Mevcut kartları veriyle güncelle
        for rov in rovlar:
            rid = rov["id"]
            if rid in self._liderler:
                self._liderler[rid].veri_guncelle(rid, rov)
            else:
                for grup in self._liderler.values():
                    if rid in grup.takipci_idleri():
                        grup.veri_guncelle(rid, rov)
                        break
            if rid in self._base:
                self._us.veri_guncelle(rid, rov)
        for grup in self._liderler.values():
            grup.stat_guncelle_veri(self._veri)
        self._stat_guncelle()

    def _ilk_yerles(self, rovlar: list[dict]):
        """İlk bağlantıda ROV'ları simülasyon state'ine göre yerleştir."""
        for r in rovlar:
            rid = r["id"]
            gps = r.get("gps", (0, 0, 0))
            self._baslangic_gps[rid] = tuple(gps)
            # İlk sim state'ini kaydet — reaktif döngünün referansı
            sim_rol = int(r.get("rol", 0) or 0)
            sim_gid = grup_id_oku(r)
            self._son_sim_state[rid] = (sim_rol, sim_gid)
        self._sim_statine_gore_yerlestir(rovlar)
        self._stat_guncelle()

    def _us_a_birak(self, rid: int):
        """ROV'u üs bölgesine al — UI konumunu günceller ve simülasyona rol sıfırlama gönderir."""
        konum, eski_lid = self._rov_konumu_bul(rid)
        if konum == "base":
            return
        if konum == "leader":
            grup = self._liderler.pop(rid, None)
            if grup:
                g_idx = grup.g_idx
                takipciler = list(grup.takipci_idleri())
                self._h_lay.removeWidget(grup)
                grup.deleteLater()
                if takipciler:
                    yeni_lid = takipciler[0]
                    self._lider_olustur(yeni_lid, emit_komut=False, sim_g_id=g_idx)
                    ng = self._liderler.get(yeni_lid)
                    if ng:
                        for tid in takipciler[1:]:
                            ng.takipci_ekle(tid, self._veri.get(tid, {"id": tid}))
        elif konum == "follower" and eski_lid is not None:
            grup = self._liderler.get(eski_lid)
            if grup:
                grup.takipci_cikar(rid)
        self._base.add(rid)
        self._us.rov_ekle(rid, self._veri.get(rid, {"id": rid}))
        if rid in self._veri:
            self._veri[rid] = {**self._veri[rid], "rol": 0, "grup_id": None}
        k = f"filo.rov_usye_al({rid})"
        komut_gonder(k)
        self.komut_uretildi.emit(k, f"ROV-{rid} → Üsse alındı")
        self._son_sim_state[rid] = (0, None)
        self._bekleyen_kaydet(rid, (0, None))
        self._stat_guncelle()

    def _gruba_birak(self, rid: int, lider_id: int):
        if rid == lider_id:
            return
        grup = self._liderler.get(lider_id)
        if grup is None or rid in grup.takipci_idleri():
            return

        if rid in self._base:
            self._base.discard(rid)
            self._us.rov_cikar(rid)
        else:
            for g in self._liderler.values():
                if rid in g.takipci_idleri():
                    g.takipci_cikar(rid)

        veri = self._veri.get(rid, {"id": rid})
        grup.takipci_ekle(rid, veri)

        g_id  = grup.g_idx
        # role=0 (takipçi) + gnc.mod=1 (takip modu) ata
        k_rol = (
            f"_r=filo.find_rov_by_id({rid});"
            f"_r and setattr(_r,'role',0);"
            f"_r and hasattr(_r,'gnc') and setattr(_r.gnc,'mod',1)"
        )
        # group_id ata + g_rovs cache sıfırla
        k_grup = (
            f"_r=filo.find_rov_by_id({rid});"
            f"_r and setattr(_r,'group_id',{g_id});"
            f"_o=getattr(filo,'ortam_ref',None);"
            f"_o and hasattr(_o,'_invalidate_g_rovs_cache') and _o._invalidate_g_rovs_cache()"
        )
        # Eski navigasyon hedefini temizle
        k_hedef = f"filo._rov_hedefleri.pop({rid},None)"
        # Varsayılan LINE formasyon (henüz yoksa)
        k_form = (
            f"isinstance(getattr(filo,'aktif_formasyon',None),dict)"
            f" or setattr(filo,'aktif_formasyon',{{}});"
            f"filo.aktif_formasyon.setdefault("
            f"{g_id},{{'id':'LINE','aralik':10,'is_3d':False,'yaw':0,'g_id':{g_id}}})"
        )
        # Grubu Lider-Takip moduna al
        k_mode = f"filo.change_mode(g_id={g_id}, new_mode=1)"

        komut_gonder(k_rol)
        komut_gonder(k_grup)
        komut_gonder(k_hedef)
        komut_gonder(k_form)
        komut_gonder(k_mode)
        # Yeni grubun aktif görevini ROV'a devret (plan varsa yeniden başlatır)
        k_gorev = f"filo.rov_gruba_gorev_aktar({rid}, {g_id})"
        komut_gonder(k_gorev)
        # UI mod etiketini güncelle (backend ile senkronize et)
        grup._mod = 1
        grup._mod_lbl.setText("⚙ Lider Takip")
        aciklama = (
            f"ROV-{rid} → Grup-{g_id} Takipçi | "
            f"role=0, group_id={g_id} | mod=1 (Lider Takip) | LINE formasyon"
        )
        self.komut_uretildi.emit(k_rol, aciklama)
        self._son_sim_state[rid] = (0, g_id)
        self._bekleyen_kaydet(rid, (0, g_id))
        self._stat_guncelle()

    # ── Lider yönetimi ────────────────────────────────────────────────────
    def _lider_olustur(self, rid: int, emit_komut: bool = True, sim_g_id: int | None = None):
        if rid in self._liderler:
            return  # zaten lider

        g_idx = sim_g_id if sim_g_id is not None else self._sonraki_grup_id()

        # Mevcut konumdan çıkar
        konum, eski_lid = self._rov_konumu_bul(rid)
        if konum == "base":
            self._base.discard(rid)
            self._us.rov_cikar(rid)
        elif konum == "follower" and eski_lid is not None and eski_lid in self._liderler:
            self._liderler[eski_lid].takipci_cikar(rid)

        # LiderGrubu widget'ını hemen oluştur ve layout'a ekle
        veri = self._veri.get(rid, {"id": rid})
        grup = LiderGrubu(rid, veri, g_idx, self.sinyal)
        grup.kaldir_istendi.connect(self._lider_kaldir)
        grup.takipci_us_istendi.connect(self._us_a_birak)
        grup.rov_birakildi.connect(lambda r, lid=rid: self._gruba_birak(r, lid))
        grup.komut_uretildi.connect(self.komut_uretildi)
        grup.menu_istendi.connect(self._menu_goster)
        self._liderler[rid] = grup
        self._h_lay.insertWidget(self._h_lay.count() - 1, grup)
        if emit_komut:
            self._bekleyen_kaydet(rid, (1, g_idx))

        if emit_komut:
            _inval = (
                "_o=getattr(filo,'ortam_ref',None);"
                "_o and hasattr(_o,'_invalidate_g_rovs_cache') and _o._invalidate_g_rovs_cache()"
            )
            # 1. Rol=1 + grup_id ata (O(1) find_rov_by_id)
            k_rol = (
                f"_r=filo.find_rov_by_id({rid});"
                f"_r and setattr(_r,'role',1);"
                f"_r and setattr(_r,'group_id',{g_idx});"
                f"{_inval}"
            )
            # 2. Otomatik lider seçimini durdur
            k_oto = (
                "hasattr(filo,'leader_manager') and "
                "setattr(filo.leader_manager,'oto_lider_etkin',False)"
            )
            # 3. Eski navigasyon hedeflerini temizle
            k_temiz = (
                f"filo._rov_hedefleri.pop({rid},None);"
                f"filo.nav_queue.pop({g_idx},None)"
            )
            # 4. GNC modunu bağımsız moda al (lider artık kimseyi takip etmiyor)
            k_gnc = (
                f"_r=filo.find_rov_by_id({rid});"
                f"_gnc=getattr(_r,'gnc',None) if _r else None;"
                f"_gnc and setattr(_gnc,'mod',0)"
            )
            # 5. Grup için varsayılan LINE formasyon (henüz yoksa)
            k_form = (
                f"isinstance(getattr(filo,'aktif_formasyon',None),dict)"
                f" or setattr(filo,'aktif_formasyon',{{}});"
                f"filo.aktif_formasyon.setdefault("
                f"{g_idx},{{'id':'LINE','aralik':10,'is_3d':False,'yaw':0,'g_id':{g_idx}}})"
            )
            # 6. Grup modunu bağımsız (0) yap
            k_mode = f"filo.change_mode(g_id={g_idx}, new_mode=0)"

            for k in (k_rol, k_oto, k_temiz, k_gnc, k_form, k_mode):
                komut_gonder(k)

            aciklama = (
                f"ROV-{rid} → LİDER (Grup-{g_idx}) | "
                f"role=1, group_id={g_idx} | mod=0 (bağımsız) | LINE formasyon | oto_lider=False"
            )
            self.komut_uretildi.emit(k_rol, aciklama)
            self._son_sim_state[rid] = (1, g_idx)
        self._stat_guncelle()

    def _lider_kaldir(self, lid: int, emit_komut: bool = True):
        grup = self._liderler.pop(lid, None)
        if grup is None:
            return
        _invalidate_cmd = (
            "_o=getattr(filo,'ortam_ref',None);"
            "_o and hasattr(_o,'_invalidate_g_rovs_cache') and _o._invalidate_g_rovs_cache()"
        )
        for tid in list(grup.takipci_idleri()):
            grup.takipci_cikar(tid)
            self._base.add(tid)
            self._us.rov_ekle(tid, self._veri.get(tid, {"id": tid}))
            if emit_komut:
                komut_gonder(f"[setattr(r,'role',0) for r in filo.rovs if getattr(r,'id',None)=={tid}]")
                komut_gonder(
                    f"_r=filo.find_rov_by_id({tid});_r and setattr(_r,'group_id',None);" + _invalidate_cmd
                )
                self._son_sim_state[tid] = (0, None)
        self._base.add(lid)
        self._us.rov_ekle(lid, self._veri.get(lid, {"id": lid}))
        g_idx = grup.g_idx
        k = f"[setattr(r,'role',0) for r in filo.rovs if getattr(r,'id',None)=={lid}]"
        k_gid = f"_r=filo.find_rov_by_id({lid});_r and setattr(_r,'group_id',None);" + _invalidate_cmd
        self._son_sim_state[lid] = (0, None)
        if emit_komut:
            komut_gonder(k)
            komut_gonder(k_gid)
            self.komut_uretildi.emit(k, f"ROV-{lid} liderliği kaldırıldı")
        self._h_lay.removeWidget(grup)
        grup.deleteLater()
        self._stat_guncelle()

    def _tum_use_al(self):
        for lid in list(self._liderler.keys()):
            self._lider_kaldir(lid)

    # ── Geni\u015fletilmi\u015f Sa\u011f T\u0131k Men\u00fcS\u00fc ───────────────────────────────────────────
    _MENU_CSS = (
        f"QMenu {{ background:#0d1520; color:{METiN}; border:1px solid {PANEL_KENAR};"
        f"  font-family:Consolas; font-size:9pt; }}"
        f"QMenu::item {{ padding:5px 22px; }}"
        f"QMenu::item:selected {{ background:#1a2535; color:{VURGU}; }}"
        f"QMenu::item:disabled {{ color:{METiN_KOYU}; }}"
        f"QMenu::separator {{ background:{PANEL_KENAR}; height:1px; margin:3px 5px; }}"
    )

    def _menu_goster(self, rid: int, pos: QPoint):
        """T\u00fcm ROV kartlar\u0131n\u0131n sa\u011f t\u0131k i\u015flem men\u00fcS\u00fc."""
        m = QMenu()
        m.setStyleSheet(self._MENU_CSS)

        hdr = m.addAction(f"\u2500\u2500  ROV-{rid}  \u2500\u2500")
        hdr.setEnabled(False)
        m.addSeparator()

        # \u2500\u2500 Lider Yap \u2500\u2500
        lider_sub = m.addMenu("\u2605  Lider Yap")
        lider_sub.setStyleSheet(self._MENU_CSS)
        a_yeni = QAction("\u2795  Yeni Grup", m)
        a_yeni.triggered.connect(lambda: self._lider_olustur(rid))
        lider_sub.addAction(a_yeni)
        # Mevcut gruplar i\u00e7in "Bu Grup" se\u00e7enekleri
        mevcut_gruplar = {lid: g for lid, g in self._liderler.items() if lid != rid}
        if mevcut_gruplar:
            lider_sub.addSeparator()
            for old_lid, grup in mevcut_gruplar.items():
                a = QAction(f"\u21ba  Bu Grup: Grup-{grup.g_idx}  (ROV-{old_lid} \u2192 takip\u00e7i)", m)
                _ol, _gi = old_lid, grup.g_idx
                a.triggered.connect(lambda _, nl=rid, ol=_ol, gi=_gi: self._lider_degistir(nl, ol, gi))
                lider_sub.addAction(a)

        # \u2500\u2500 Takip\u00e7i Yap \u2500\u2500
        tak_sub = m.addMenu("\u2694  Takip\u00e7i Yap")
        tak_sub.setStyleSheet(self._MENU_CSS)
        secenekler = []
        for lid, grup in self._liderler.items():
            if lid == rid or rid in grup.takipci_idleri():
                continue
            a = QAction(f"Grup-{grup.g_idx}  (ROV-{lid})", m)
            _l = lid
            a.triggered.connect(lambda _, l=_l: self._gruba_birak(rid, l))
            tak_sub.addAction(a)
            secenekler.append(a)
        if not secenekler:
            na = QAction("(Hen\u00fcz grup yok)", m)
            na.setEnabled(False)
            tak_sub.addAction(na)

        m.addSeparator()

        # ── Grup Modu (ROV bir grupta ise) ──
        konum, eski_lid = self._rov_konumu_bul(rid)
        if konum == "leader":
            grup_widget = self._liderler.get(rid)
        elif konum == "follower" and eski_lid is not None:
            grup_widget = self._liderler.get(eski_lid)
        else:
            grup_widget = None
        if grup_widget is not None:
            mod_sub = m.addMenu("\u2699  Grup Modu")
            mod_sub.setStyleSheet(self._MENU_CSS)
            a_bag = QAction("\u25a0  Ba\u011f\u0131ms\u0131z" + ("  \u2714" if grup_widget._mod == 0 else ""), m)
            _gw = grup_widget
            a_bag.triggered.connect(lambda: _gw._mod_uygula(0))
            a_tak = QAction("\u25b6  Lider Takip" + ("  \u2714" if grup_widget._mod == 1 else ""), m)
            a_tak.triggered.connect(lambda: _gw._mod_uygula(1))
            mod_sub.addAction(a_bag)
            mod_sub.addAction(a_tak)
            m.addSeparator()

        # \u2500\u2500 \u00dcsse G\u00f6nder \u2500\u2500
        a_us = QAction("\ud83c\udfe0  \u00dcsse G\u00f6nder (ba\u015flang\u0131\u00e7 noktas\u0131)", m)
        a_us.triggered.connect(lambda: self._us_gonder(rid))
        m.addAction(a_us)

        # Simülasyondan Çıkart
        m.addSeparator()
        a_cikar = QAction("\ud83d\uddd1  Sim\u00fclasyondan \u00c7\u0131kart", m)
        a_cikar.triggered.connect(lambda: self._simden_cikar(rid))
        m.addAction(a_cikar)

        m.exec_(pos)

    def _us_gonder(self, rid: int):
        """ROV'u ba\u015flang\u0131\u00e7 konumuna g\u00f6nder ve \u00fcsse al."""
        self._us_a_birak(rid)
        gps0 = self._baslangic_gps.get(rid)
        if gps0 and len(gps0) >= 2:
            x, y, z = float(gps0[0]), float(gps0[1]), float(gps0[2]) if len(gps0) > 2 else 0.0
            k = f"filo.git(rov_id={rid}, x={x}, y={y}, z={z}, ai=True)"
            komut_gonder(k)
            self.komut_uretildi.emit(k, f"ROV-{rid} \u2192 Ba\u015flang\u0131\u00e7 noktas\u0131na d\u00f6n\u00fcyor")

    def _simden_cikar(self, rid: int):
        """ROV'u UI'dan kald\u0131r ve sim\u00fclasyona \u00e7\u0131kart komutu g\u00f6nder."""
        if rid in self._base:
            self._base.discard(rid)
            self._us.rov_cikar(rid)
        elif rid in self._liderler:
            self._lider_kaldir(rid)
        else:
            for g in list(self._liderler.values()):
                if rid in g.takipci_idleri():
                    g.takipci_cikar(rid)
        self._veri.pop(rid, None)
        self._baslangic_gps.pop(rid, None)
        k = f"ui_rov_cikar({int(rid)})"
        komut_gonder(k)
        self.komut_uretildi.emit(k, f"ROV-{rid} sim\u00fclasyondan \u00e7\u0131kart\u0131ld\u0131")
        self._stat_guncelle()

    def _sim_rov_ekle(self):
        """Sim\u00fclasyona yeni ROV ekle."""
        dlg = QDialog(self)
        dlg.setWindowTitle("ROV Ekle")
        dlg.setStyleSheet(f"QDialog {{ background:#0a0d12; color:{METiN}; }} QLabel {{ color:{METiN}; }}")
        form = QFormLayout(dlg)

        dbl = QDoubleValidator(-9999.0, 9999.0, 2)

        def _line(vars: str, validator=None):
            edit = QLineEdit(vars)
            edit.setStyleSheet(_INPUT_CSS)
            if validator is not None:
                edit.setValidator(validator)
            return edit

        x_edit = _line("0", dbl)
        y_edit = _line("-10", dbl)
        z_edit = _line("0", dbl)
        manuel_check = QCheckBox("Manuel koordinat kullan")
        manuel_check.setChecked(False)
        manuel_check.setToolTip("Kapalıyken ROV otomatik olarak ileri karakol bölgesinde doğar")
        manuel_check.setStyleSheet(f"""
            QCheckBox {{
                color:{SARI};
                font-family: Consolas;
                font-size: 9pt;
                font-weight: bold;
                padding: 6px;
                border: 1px solid {PANEL_KENAR};
                border-radius: 4px;
                background: #111821;
            }}
            QCheckBox::indicator {{
                width: 16px;
                height: 16px;
            }}
        """)
        for edit in (x_edit, y_edit, z_edit):
            edit.setEnabled(False)
        manuel_check.toggled.connect(lambda aktif: [edit.setEnabled(aktif) for edit in (x_edit, y_edit, z_edit)])
        group_edit = _line("", QIntValidator(0, 999999))
        group_edit.setPlaceholderText("Boş = üs (grupsuz); gruplar 0'dan başlar")
        model_edit = _line("submarine")
        rol_combo = QComboBox()
        rol_combo.addItem("Takipçi / normal (rol=0)", 0)
        rol_combo.addItem("Lider (rol=1)", 1)
        rol_combo.setStyleSheet(_COMBO_CSS)

        form.addRow("", manuel_check)
        form.addRow("X", x_edit)
        form.addRow("Y", y_edit)
        form.addRow("Z", z_edit)
        form.addRow("Grup ID", group_edit)
        form.addRow("Model", model_edit)
        form.addRow("Rol", rol_combo)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        form.addRow(buttons)
        if dlg.exec_() != QDialog.Accepted:
            return

        def _float(edit: QLineEdit, default: float) -> float:
            text = edit.text().strip().replace(",", ".")
            return float(text) if text else default

        group_text = group_edit.text().strip()
        group_id = int(group_text) if group_text else None
        model_key = model_edit.text().strip() or "submarine"
        rol = int(rol_combo.currentData())
        position_arg = ""
        if manuel_check.isChecked():
            position = (_float(x_edit, 0.0), _float(y_edit, -10.0), _float(z_edit, 0.0))
            position_arg = f", position={position!r}"
        k = f"ui_rov_ekle(group_id={group_id!r}{position_arg}, model_key={model_key!r}, rol={rol})"
        komut_gonder(k)
        self.komut_uretildi.emit(k, "Yeni ROV sim\u00fclasyona ekleniyor...")

    def _lider_degistir(self, yeni_lid: int, eski_lid: int, g_idx: int, emit_komut: bool = True):
        """Grup liderini de\u011fi\u015ftir: eski lider tak\u0131p\u00e7i olur."""
        grup = self._liderler.get(eski_lid)
        if grup is None:
            return
        # yeni_lid'i mevcut konumundan \u00e7\u0131kar
        if yeni_lid in self._base:
            self._base.discard(yeni_lid)
            self._us.rov_cikar(yeni_lid)
        else:
            for g in self._liderler.values():
                if yeni_lid in g.takipci_idleri():
                    g.takipci_cikar(yeni_lid)
        # Eski grubun takip\u00e7ilerini kaydet ve widget'i y\u0131k
        takipciler = list(grup.takipci_idleri())
        for tid in takipciler:
            grup.takipci_cikar(tid)
        self._liderler.pop(eski_lid)
        self._h_lay.removeWidget(grup)
        grup.deleteLater()
        # Ayn\u0131 g_idx ile yeni liderle grup olu\u015ftur
        yeni_veri = self._veri.get(yeni_lid, {"id": yeni_lid})
        eski_veri = self._veri.get(eski_lid, {"id": eski_lid})
        new_grup = LiderGrubu(yeni_lid, yeni_veri, g_idx, self.sinyal)
        new_grup.kaldir_istendi.connect(self._lider_kaldir)
        new_grup.takipci_us_istendi.connect(self._us_a_birak)
        new_grup.rov_birakildi.connect(lambda r, lid=yeni_lid: self._gruba_birak(r, lid))
        new_grup.komut_uretildi.connect(self.komut_uretildi)
        new_grup.menu_istendi.connect(self._menu_goster)
        self._liderler[yeni_lid] = new_grup
        self._h_lay.insertWidget(self._h_lay.count() - 1, new_grup)
        # Eski lideri + tak\u0131p\u00e7ileri yeni gruba ekle
        new_grup.takipci_ekle(eski_lid, eski_veri)
        for tid in takipciler:
            new_grup.takipci_ekle(tid, self._veri.get(tid, {"id": tid}))
        # Komutlar
        if emit_komut:
            komut_gonder(f"[setattr(r,'role',1) for r in filo.rovs if getattr(r,'id',None)=={yeni_lid}]")
            komut_gonder(f"[setattr(r,'role',0) for r in filo.rovs if getattr(r,'id',None)=={eski_lid}]")
            self.komut_uretildi.emit(
                f"ROV-{yeni_lid} \u2192 Grup-{g_idx} Lider",
                f"Grup-{g_idx}: ROV-{eski_lid} \u2192 tak\u0131p\u00e7i"
            )
        self._stat_guncelle()

    def _stat_guncelle(self):
        t = len(self._veri)
        l = len(self._liderler)
        u = len(self._base)
        # Takipçi sayısı: tüm grupların takipçi toplamı
        g = sum(len(grup.takipci_idleri()) for grup in self._liderler.values())
        self._stat.setText(
            f"Toplam: {t} ROV  |  Lider: {l}  |  Üste: {u}  |  Takipçi: {g}"
        )
