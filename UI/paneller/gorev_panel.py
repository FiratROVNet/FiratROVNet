"""
Görev Paneli — Alan Tarama, Arama Kurtarma, İmha görevlerini yapılandırır ve başlatır.
"""
from __future__ import annotations

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QSpinBox, QDoubleSpinBox, QPushButton, QGridLayout,
    QLineEdit, QComboBox, QTabWidget, QFrame, QCheckBox,
    QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont

from UI.tema import (
    VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI, TURUNCU,
    PANEL, PANEL_KENAR, GOREV_RENK,
)
from UI.kopru import komut_gonder


# ── Yardımcı ─────────────────────────────────────────────────────────────────
def _etiket(metin: str, renk: str = METiN_KOYU) -> QLabel:
    lbl = QLabel(metin)
    lbl.setStyleSheet(f"color: {renk}; font-size: 9pt;")
    return lbl


def _alan_widget() -> tuple[QWidget, callable]:
    """(x_min, y_min, x_max, y_max) alan giriş grubu döner."""
    w = QWidget()
    lay = QGridLayout(w)
    lay.setContentsMargins(0, 0, 0, 0)
    lay.setSpacing(4)

    spins = {}
    tanimlar = [
        ("x_min", 0, 0, -500, 500),
        ("y_min", 0, 1, -500, 500),
        ("x_max", 1, 0, -500, 500),
        ("y_max", 1, 1, -500, 500),
    ]
    varsayilan = {"x_min": 0, "y_min": 0, "x_max": 200, "y_max": 200}

    for isim, satir, sutun, alt, ust in tanimlar:
        spin = QDoubleSpinBox()
        spin.setRange(alt, ust)
        spin.setValue(varsayilan[isim])
        spin.setPrefix(f"{isim}: ")
        spin.setSingleStep(10)
        lay.addWidget(spin, satir, sutun)
        spins[isim] = spin

    def deger_al():
        return (
            spins["x_min"].value(), spins["y_min"].value(),
            spins["x_max"].value(), spins["y_max"].value(),
        )

    return w, deger_al


# ── Alan Tarama Sekmesi ───────────────────────────────────────────────────────
class AlanTaramaSekmesi(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        # Alan
        alan_kutu = QGroupBox("TARAMA ALANI")
        alan_lay  = QVBoxLayout(alan_kutu)
        self._alan_w, self._alan_al = _alan_widget()
        alan_lay.addWidget(self._alan_w)

        # Z derinliği satırı
        z_lay = QHBoxLayout()
        z_lay.addWidget(_etiket("Derinlik (m, negatif):"))
        self.spin_derinlik = QDoubleSpinBox()
        self.spin_derinlik.setRange(-300, 0)
        self.spin_derinlik.setValue(-20.0)
        self.spin_derinlik.setSingleStep(1.0)
        z_lay.addWidget(self.spin_derinlik)
        alan_lay.addLayout(z_lay)
        lay.addWidget(alan_kutu)

        # Parametreler
        param_kutu = QGroupBox("PARAMETRELER")
        param_lay  = QGridLayout(param_kutu)
        param_lay.setSpacing(8)
        param_lay.setColumnStretch(1, 1)

        param_lay.addWidget(_etiket("Grup ID"), 0, 0)
        self.spin_grup = QSpinBox(); self.spin_grup.setRange(0, 9)
        param_lay.addWidget(self.spin_grup, 0, 1)

        param_lay.addWidget(_etiket("Şerit Aralığı (m)"), 1, 0)
        self.spin_serit = QDoubleSpinBox()
        self.spin_serit.setRange(3.0, 100.0); self.spin_serit.setValue(15.0)
        param_lay.addWidget(self.spin_serit, 1, 1)

        param_lay.addWidget(_etiket("ROV Sayısı (0=hepsi)"), 2, 0)
        self.spin_rov_say = QSpinBox(); self.spin_rov_say.setRange(0, 20)
        self.spin_rov_say.setToolTip("0 → gruptaki tüm idle ROV'lar kullanılır")
        param_lay.addWidget(self.spin_rov_say, 2, 1)

        self.chk_sessiz = QCheckBox("Sessiz mod (log kapalı)")
        self.chk_sessiz.setChecked(True)
        param_lay.addWidget(self.chk_sessiz, 3, 0, 1, 2)

        lay.addWidget(param_kutu)

        # Buton grubu
        btn_lay = QHBoxLayout()
        btn_basla = QPushButton("▶  Alan Taramayı Başlat")
        btn_basla.setObjectName("btn_basla")
        btn_basla.clicked.connect(self._basla)
        btn_durdur = QPushButton("■  Durdur")
        btn_durdur.setObjectName("btn_durdur")
        btn_durdur.clicked.connect(self._durdur)
        btn_lay.addWidget(btn_basla)
        btn_lay.addWidget(btn_durdur)
        lay.addLayout(btn_lay)
        lay.addStretch()

    def _basla(self):
        x1, y1, x2, y2 = self._alan_al()
        g_id    = self.spin_grup.value()
        derinlik = self.spin_derinlik.value()
        serit   = self.spin_serit.value()
        n_rov   = self.spin_rov_say.value()
        sessiz  = self.chk_sessiz.isChecked()

        n_rov_str = f"gereken_rov_sayisi={n_rov}, " if n_rov > 0 else ""
        komut = (
            f"filo.alan_tarama_gorevi.baslat("
            f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), "
            f"derinlik={derinlik}, serit_araligi={serit}, "
            f"{n_rov_str}sessiz={sessiz})"
        )
        aciklama = f"Alan Tarama → Grup-{g_id} | Alan:({x1},{y1})–({x2},{y2}) | D:{derinlik}m"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)

    def _durdur(self):
        g_id  = self.spin_grup.value()
        komut = f"filo.alan_tarama_gorevi.durdur(grup_id={g_id})"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "warn"))
        self.komut_uretildi.emit(komut, f"Alan Tarama Durduruldu → Grup-{g_id}")


# ── Arama Kurtarma Sekmesi ────────────────────────────────────────────────────
class AramaKurtarmaSekmesi(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        alan_kutu = QGroupBox("ARAMA ALANI")
        alan_lay  = QVBoxLayout(alan_kutu)
        self._alan_w, self._alan_al = _alan_widget()
        alan_lay.addWidget(self._alan_w)
        lay.addWidget(alan_kutu)

        param_kutu = QGroupBox("PARAMETRELER")
        param_lay  = QGridLayout(param_kutu)
        param_lay.setSpacing(8)
        param_lay.setColumnStretch(1, 1)

        param_lay.addWidget(_etiket("Grup ID"), 0, 0)
        self.spin_grup = QSpinBox(); self.spin_grup.setRange(0, 9)
        param_lay.addWidget(self.spin_grup, 0, 1)

        param_lay.addWidget(_etiket("Derinlik (m)"), 1, 0)
        self.spin_derinlik = QDoubleSpinBox()
        self.spin_derinlik.setRange(-300, 0); self.spin_derinlik.setValue(-20.0)
        param_lay.addWidget(self.spin_derinlik, 1, 1)

        param_lay.addWidget(_etiket("Hedef Sınıflar (virgülle)"), 2, 0)
        self.txt_sinif = QLineEdit("person")
        self.txt_sinif.setToolTip("YOLO sınıf isimleri, örn: person, boat, diver")
        param_lay.addWidget(self.txt_sinif, 2, 1)

        param_lay.addWidget(_etiket("Güven Eşiği"), 3, 0)
        self.spin_guven = QDoubleSpinBox()
        self.spin_guven.setRange(0.1, 1.0); self.spin_guven.setValue(0.5)
        self.spin_guven.setSingleStep(0.05)
        param_lay.addWidget(self.spin_guven, 3, 1)

        param_lay.addWidget(_etiket("YOLO Model"), 4, 0)
        self.txt_model = QLineEdit("yolov8n.pt")
        param_lay.addWidget(self.txt_model, 4, 1)

        lay.addWidget(param_kutu)

        btn_lay = QHBoxLayout()
        btn_basla = QPushButton("▶  Aramayı Başlat")
        btn_basla.setObjectName("btn_basla")
        btn_basla.clicked.connect(self._basla)
        btn_durdur = QPushButton("■  Durdur")
        btn_durdur.setObjectName("btn_durdur")
        btn_durdur.clicked.connect(self._durdur)
        btn_lay.addWidget(btn_basla)
        btn_lay.addWidget(btn_durdur)
        lay.addLayout(btn_lay)
        lay.addStretch()

    def _basla(self):
        x1, y1, x2, y2 = self._alan_al()
        g_id     = self.spin_grup.value()
        derinlik = self.spin_derinlik.value()
        guven    = self.spin_guven.value()
        model    = self.txt_model.text().strip() or "yolov8n.pt"
        siniflar_raw = [s.strip() for s in self.txt_sinif.text().split(",") if s.strip()]
        siniflar_str = repr(siniflar_raw) if siniflar_raw else "None"

        komut = (
            f"filo.arama_kurtarma_gorevi.baslat("
            f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), "
            f"hedef_siniflari={siniflar_str}, model_path='{model}', "
            f"derinlik={derinlik}, min_confidence={guven})"
        )
        aciklama = f"Arama Kurtarma → Grup-{g_id} | Hedef: {siniflar_raw}"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)

    def _durdur(self):
        komut = "filo.arama_kurtarma_gorevi.durdur()"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "warn"))
        self.komut_uretildi.emit(komut, "Arama Kurtarma Durduruldu")


# ── İmha Sekmesi ──────────────────────────────────────────────────────────────
class ImhaSekmesi(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        # Mod seçimi
        mod_kutu = QGroupBox("İMHA MODU")
        mod_lay  = QVBoxLayout(mod_kutu)
        self.cmb_mod = QComboBox()
        self.cmb_mod.addItem("Koordinat ile İmha (Bilinen Hedef)", "koordinat")
        self.cmb_mod.addItem("Alan Tarayarak İmha (YOLO ile Bul)", "alan")
        self.cmb_mod.currentIndexChanged.connect(self._mod_degisti)
        mod_lay.addWidget(self.cmb_mod)
        lay.addWidget(mod_kutu)

        # ── Koordinat modu ──
        self.w_koordinat = QGroupBox("HEDEF KOORDİNATI")
        k_lay = QGridLayout(self.w_koordinat)
        k_lay.setSpacing(8)
        k_lay.setColumnStretch(1, 1)

        k_lay.addWidget(_etiket("Grup ID"), 0, 0)
        self.spin_k_grup = QSpinBox(); self.spin_k_grup.setRange(0, 9)
        k_lay.addWidget(self.spin_k_grup, 0, 1)

        for i, (lbl, attr, val) in enumerate([
            ("Hedef X", "spin_hx", 100.0),
            ("Hedef Y", "spin_hy", 80.0),
            ("Hedef Z", "spin_hz", -15.0),
        ], start=1):
            k_lay.addWidget(_etiket(lbl), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-500, 500); spin.setValue(val)
            k_lay.addWidget(spin, i, 1)
            setattr(self, attr, spin)

        k_lay.addWidget(_etiket("İmha Mesafesi (m)"), 4, 0)
        self.spin_k_mesafe = QDoubleSpinBox()
        self.spin_k_mesafe.setRange(1.0, 50.0); self.spin_k_mesafe.setValue(8.0)
        k_lay.addWidget(self.spin_k_mesafe, 4, 1)
        lay.addWidget(self.w_koordinat)

        # ── Alan modu ──
        self.w_alan = QGroupBox("ALAN VE HEDEF SINIF")
        a_lay = QVBoxLayout(self.w_alan)
        self._alan_w, self._alan_al = _alan_widget()
        a_lay.addWidget(self._alan_w)

        a2_lay = QGridLayout()
        a2_lay.setSpacing(8)
        a2_lay.setColumnStretch(1, 1)
        a2_lay.addWidget(_etiket("Grup ID"), 0, 0)
        self.spin_a_grup = QSpinBox(); self.spin_a_grup.setRange(0, 9)
        a2_lay.addWidget(self.spin_a_grup, 0, 1)
        a2_lay.addWidget(_etiket("Hedef Sınıflar"), 1, 0)
        self.txt_a_sinif = QLineEdit("mine, bomb")
        a2_lay.addWidget(self.txt_a_sinif, 1, 1)
        a2_lay.addWidget(_etiket("İmha Mesafesi (m)"), 2, 0)
        self.spin_a_mesafe = QDoubleSpinBox()
        self.spin_a_mesafe.setRange(1.0, 50.0); self.spin_a_mesafe.setValue(8.0)
        a2_lay.addWidget(self.spin_a_mesafe, 2, 1)
        a_lay.addLayout(a2_lay)
        lay.addWidget(self.w_alan)
        self.w_alan.setVisible(False)

        # Butonlar
        btn_lay = QHBoxLayout()
        btn_basla = QPushButton("▶  İmhayı Başlat")
        btn_basla.setObjectName("btn_durdur")   # kırmızı stil
        btn_basla.clicked.connect(self._basla)
        btn_guncelle = QPushButton("↻  Sonucu Sorgula")
        btn_guncelle.clicked.connect(self._guncelle)
        btn_lay.addWidget(btn_basla)
        btn_lay.addWidget(btn_guncelle)
        lay.addLayout(btn_lay)
        lay.addStretch()

    def _mod_degisti(self, idx):
        self.w_koordinat.setVisible(idx == 0)
        self.w_alan.setVisible(idx == 1)

    def _basla(self):
        mod = self.cmb_mod.currentData()
        if mod == "koordinat":
            g_id    = self.spin_k_grup.value()
            hedef   = (self.spin_hx.value(), self.spin_hy.value(), self.spin_hz.value())
            mesafe  = self.spin_k_mesafe.value()
            komut   = (f"filo.imha_gorevi.koordinat_imha_baslat("
                       f"grup_id={g_id}, hedef={hedef}, imha_mesafesi={mesafe})")
            aciklama = f"İmha → Grup-{g_id} | Hedef:{hedef} | Mesafe:{mesafe}m"
        else:
            x1, y1, x2, y2 = self._alan_al()
            g_id    = self.spin_a_grup.value()
            siniflar_raw = [s.strip() for s in self.txt_a_sinif.text().split(",") if s.strip()]
            mesafe  = self.spin_a_mesafe.value()
            komut   = (f"filo.imha_gorevi.alan_imha_baslat("
                       f"grup_id={g_id}, alan=({x1},{y1},{x2},{y2}), "
                       f"hedef_siniflari={repr(siniflar_raw)}, imha_mesafesi={mesafe})")
            aciklama = f"Alan İmha → Grup-{g_id} | Hedef:{siniflar_raw}"

        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "err"))
        self.komut_uretildi.emit(komut, aciklama)

    def _guncelle(self):
        komut = "sonuc = filo.imha_gorevi.guncelle(); print(sonuc)"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "warn"))
        self.komut_uretildi.emit(komut, "İmha sonucu sorgulandı")


# ── Hareket Sekmesi (Git / Move) ──────────────────────────────────────────────
class HareketSekmesi(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal, rov_panel_ref=None, parent=None):
        super().__init__(parent)
        self.sinyal        = sinyal
        self.rov_panel_ref = rov_panel_ref
        lay = QVBoxLayout(self)
        lay.setSpacing(10)

        # Tek nokta git
        git_kutu = QGroupBox("HEDEFE GİT")
        git_lay  = QGridLayout(git_kutu)
        git_lay.setSpacing(8)
        git_lay.setColumnStretch(1, 1)

        git_lay.addWidget(_etiket("ROV ID"), 0, 0)
        self.spin_git_rov = QSpinBox(); self.spin_git_rov.setRange(0, 20)
        git_lay.addWidget(self.spin_git_rov, 0, 1)

        for i, (lbl, attr, val) in enumerate([
            ("X (sağ)", "spin_gx", 100.0),
            ("Y (ileri)", "spin_gy", 50.0),
            ("Z (derinlik)", "spin_gz", -20.0),
        ], start=1):
            git_lay.addWidget(_etiket(lbl), i, 0)
            spin = QDoubleSpinBox()
            spin.setRange(-500, 500); spin.setValue(val)
            git_lay.addWidget(spin, i, 1)
            setattr(self, attr, spin)

        self.chk_ai = QCheckBox("AI (GAT) Yol Planlaması")
        self.chk_ai.setChecked(True)
        git_lay.addWidget(self.chk_ai, 4, 0, 1, 2)

        btn_git = QPushButton("→  ROV'u Hedefe Gönder")
        btn_git.setObjectName("btn_basla")
        btn_git.clicked.connect(self._git)
        git_lay.addWidget(btn_git, 5, 0, 1, 2)

        lay.addWidget(git_kutu)

        # Grup hedefe git
        grup_kutu = QGroupBox("GRUBU HEDEFE GÖNDER")
        grup_lay  = QGridLayout(grup_kutu)
        grup_lay.setSpacing(8)
        grup_lay.setColumnStretch(1, 1)

        grup_lay.addWidget(_etiket("Grup ID"), 0, 0)
        self.spin_gg_grup = QSpinBox(); self.spin_gg_grup.setRange(0, 9)
        grup_lay.addWidget(self.spin_gg_grup, 0, 1)

        for i, (lbl, attr, val) in enumerate([
            ("X", "spin_ggx", 100.0),
            ("Y", "spin_ggy", 100.0),
            ("Z", "spin_ggz", -15.0),
        ], start=1):
            grup_lay.addWidget(_etiket(lbl), i, 0)
            spin = QDoubleSpinBox(); spin.setRange(-500, 500); spin.setValue(val)
            grup_lay.addWidget(spin, i, 1)
            setattr(self, attr, spin)

        btn_grup_git = QPushButton("→→  Grubu Hedefe Gönder")
        btn_grup_git.setObjectName("btn_basla")
        btn_grup_git.clicked.connect(self._grup_git)
        grup_lay.addWidget(btn_grup_git, 4, 0, 1, 2)

        lay.addWidget(grup_kutu)
        lay.addStretch()

    def _git(self):
        rid = self.spin_git_rov.value()
        x, y, z = self.spin_gx.value(), self.spin_gy.value(), self.spin_gz.value()
        ai = self.chk_ai.isChecked()
        komut    = f"filo.git(rov_id={rid}, x={x}, y={y}, z={z}, ai={ai})"
        aciklama = f"ROV-{rid} → ({x}, {y}, {z})"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)

    def _grup_git(self):
        g_id = self.spin_gg_grup.value()
        x, y, z = self.spin_ggx.value(), self.spin_ggy.value(), self.spin_ggz.value()
        # filo.git_grup() yok — grup ROV'larını tek tek hedefe gönder
        komut    = (
            f"[filo.git(rov_id=r.id, x={x}, y={y}, z={z}, ai=True) "
            f"for r in (filo.g_rovs.get({g_id}) or []) if r]"
        )
        aciklama = f"Grup-{g_id} → ({x}, {y}, {z})"
        komut_gonder(komut, callback=lambda s: self.sinyal.durum_guncellendi.emit(s, "ok"))
        self.komut_uretildi.emit(komut, aciklama)


# ── Ana Görev Paneli ──────────────────────────────────────────────────────────
class GorevPanel(QWidget):
    komut_uretildi = pyqtSignal(str, str)

    def __init__(self, sinyal=None, rov_panel_ref=None, grup_panel_ref=None, parent=None):
        super().__init__(parent)
        self.sinyal         = sinyal
        self.rov_panel_ref  = rov_panel_ref
        self.grup_panel_ref = grup_panel_ref

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)

        self.sekme = QTabWidget()

        self.alan_sekme     = AlanTaramaSekmesi(sinyal)
        self.arama_sekme    = AramaKurtarmaSekmesi(sinyal)
        self.imha_sekme     = ImhaSekmesi(sinyal)
        self.hareket_sekme  = HareketSekmesi(sinyal, rov_panel_ref=rov_panel_ref)

        self.sekme.addTab(self.hareket_sekme, "🧭 Hareket")
        self.sekme.addTab(self.alan_sekme,    "🗺 Alan Tarama")
        self.sekme.addTab(self.arama_sekme,   "🔍 Arama Kurtarma")
        self.sekme.addTab(self.imha_sekme,    "💥 İmha")

        # Alt panellerin sinyallerini üst panele ilet
        for sekme in (self.alan_sekme, self.arama_sekme,
                      self.imha_sekme, self.hareket_sekme):
            sekme.komut_uretildi.connect(self.komut_uretildi)

        lay.addWidget(self.sekme)
