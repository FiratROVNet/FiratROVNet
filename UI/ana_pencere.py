"""
FiratROVNet Komuta Merkezi — Ana Pencere
Askeri tarzda ROV filo yönetim arayüzü.
"""

from __future__ import annotations
import sys
import os

try:
    from PyQt5.QtCore import QLibraryInfo
    _QT_PLUGINS = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    if os.path.isdir(_QT_PLUGINS):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _QT_PLUGINS
        os.environ["QT_PLUGIN_PATH"] = _QT_PLUGINS
except Exception:
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    os.environ.pop("QT_PLUGIN_PATH", None)

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QTabWidget, QStatusBar, QLabel, QFrame, QPushButton,
    QDialog, QTextBrowser,
)
from PyQt5.QtCore import Qt, QTimer, QSettings, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QIcon, QColor, QPalette

# UI paket kökü sys.path'e ekle
_UI_DIR = os.path.dirname(__file__)
_ROOT   = os.path.dirname(_UI_DIR)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from UI.tema import GLOBAL_QSS, VURGU, METiN, ARKA_PLAN, PANEL, YESiL, KIRMIZI, SARI, METiN_KOYU
from UI.kopru import rov_listesi, bagli_mi, sim_bagli_mi

from UI.paneller.rov_panel       import ROVPanel
from UI.paneller.gorev_panel     import GorevPanel
from UI.paneller.komut_panel     import KomutPanel
from UI.paneller.surucu_panel    import SurucuPanel


# ── İşaretçi: Simülasyon olayları için sinyal köprüsü ────────────────────────
class SinyalKoprusu(QObject):
    durum_guncellendi = pyqtSignal(str, str)   # (mesaj, seviye)  seviye: ok|warn|err


# ── Kılavuz Diyaloğu ─────────────────────────────────────────────────────────
class KilavuzDialog(QDialog):
    ICERIK = """
<style>
body  { background:#0d1117; color:#cfd8dc; font-family:monospace; font-size:10pt; }
h2    { color:#00e5ff; border-bottom:1px solid #1e2d40; padding-bottom:4px; }
h3    { color:#80cbc4; margin-top:14px; }
table { border-collapse:collapse; width:100%; margin-top:6px; }
th    { background:#1a2535; color:#00e5ff; padding:5px 10px; text-align:left; }
td    { padding:4px 10px; border-bottom:1px solid #1e2d40; }
td:first-child { color:#ffd740; white-space:nowrap; }
</style>
<body>

<h2>⬡ FiratROVNet — Kullanım Kılavuzu</h2>

<h3>ROV Paneli (sol sütun)</h3>
<table>
<tr><th>Buton</th><th>Ne yapar</th></tr>
<tr><td>⭐ Lider Yap</td><td>Seçili ROV'u grubun lideri yapar (rol=1, gnc.mod=1)</td></tr>
<tr><td>↩ Üsse Bırak</td><td>Seçili ROV'u gruptan çıkarır, bağımsız moda alır (gnc.mod=0)</td></tr>
</table>

<h3>SÜRÜ Paneli — Grup Yönetimi</h3>
<table>
<tr><th>Buton / Alan</th><th>Ne yapar</th></tr>
<tr><td>Grup Oluştur</td><td>Yeni boş grup açar (grup listesine ekler)</td></tr>
<tr><td>ROV → Gruba Bırak</td><td>Sürükle-bırak: ROV'u gruba atar, otomatik LINE formasyonu başlar</td></tr>
<tr><td>Lider Oluştur</td><td>Gruptaki ilk ROV'u lider seçer</td></tr>
<tr><td>Lideri Kaldır</td><td>Grubun liderini sıfırlar (rol=0)</td></tr>
</table>

<h3>SÜRÜ Paneli — Formasyon</h3>
<table>
<tr><th>Formasyon</th><th>Ne yapar</th></tr>
<tr><td>LINE</td><td>Yan yana yatay sıra (varsayılan)</td></tr>
<tr><td>V</td><td>V şekli, lider önde</td></tr>
<tr><td>CIRCLE</td><td>Lider etrafında çember</td></tr>
<tr><td>DIAMOND</td><td>Baklava/elmas düzeni</td></tr>
<tr><td>Aralık (m)</td><td>ROVlar arası mesafeyi ayarlar</td></tr>
<tr><td>Uygula</td><td>Seçili formasyonu aktif gruba yazar, takip başlar</td></tr>
</table>

<h3>GÖREV Paneli</h3>
<table>
<tr><th>Buton</th><th>Ne yapar</th></tr>
<tr><td>Göreve Git</td><td>Seçili ROV'u girilen (x,y,z) hedefine gönderir</td></tr>
<tr><td>Dur</td><td>ROV'u durdurur (hedefi temizler)</td></tr>
<tr><td>AI Modu</td><td>Hedefe giderken GAT yapay zekasını etkinleştirir</td></tr>
</table>

<h3>KOMUT Paneli (sağ sütun)</h3>
<table>
<tr><th>Alan</th><th>Ne yapar</th></tr>
<tr><td>Komut Geçmişi</td><td>Gönderilen tüm komutların çıktısını gösterir</td></tr>
<tr><td>Komut Girişi</td><td>Doğrudan Python komutu gönderir (filo, app erişimi var)</td></tr>
<tr><td>Gönder</td><td>Komutu simülasyona iletir</td></tr>
</table>

<h3>Klavye Kısayolları (simülasyon penceresi)</h3>
<table>
<tr><th>Tuş</th><th>Ne yapar</th></tr>
<tr><td>Tab</td><td>İzlenen grubu değiştirir</td></tr>
<tr><td>P</td><td>Aktif grubun liderini patlatır (hasar testi)</td></tr>
<tr><td>G</td><td>GAT yapay zekasını aç/kapat</td></tr>
<tr><td>F</td><td>Ekran görüntüsü alır</td></tr>
<tr><td>R</td><td>Rerun kaydını başlatır/durdurur</td></tr>
<tr><td>I → Enter</td><td>Canlı Python konsolunu açar</td></tr>
</table>

</body>
"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Kullanım Kılavuzu")
        self.resize(700, 600)
        self.setStyleSheet("background:#0d1117;")
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)

        tb = QTextBrowser()
        tb.setOpenExternalLinks(False)
        tb.setHtml(self.ICERIK)
        tb.setStyleSheet("background:#0d1117; border:none;")
        lay.addWidget(tb)

        btn_kapat = QPushButton("Kapat")
        btn_kapat.setFixedHeight(30)
        btn_kapat.setStyleSheet(f"""
            QPushButton {{
                background:#1a2535; color:#cfd8dc;
                border:1px solid #1e2d40; border-radius:4px; font-size:9pt;
            }}
            QPushButton:hover {{ color:#00e5ff; border-color:#00e5ff; }}
        """)
        btn_kapat.clicked.connect(self.accept)
        lay.addWidget(btn_kapat)


# ── Üst Başlık Çubuğu ────────────────────────────────────────────────────────
class BaslikCubugu(QFrame):
    duzen_sifirla = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(52)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: #080c11;
                border-bottom: 2px solid {VURGU};
            }}
        """)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 0, 16, 0)

        # Logo / İsim
        lbl_logo = QLabel("⬡")
        lbl_logo.setStyleSheet(f"color: {VURGU}; font-size: 22pt; background: transparent;")
        lbl_isim = QLabel("FiratROVNet  <span style='color:#546e7a; font-size:10pt;'>KOMUTA MERKEZİ</span>")
        lbl_isim.setTextFormat(Qt.RichText)
        lbl_isim.setStyleSheet("color: #cfd8dc; font-size: 14pt; font-weight: bold; background: transparent;")

        lay.addWidget(lbl_logo)
        lay.addWidget(lbl_isim)
        lay.addStretch()

        # Bağlantı durumu
        self.lbl_baglanti = QLabel("● BAĞLANTI YOK")
        self.lbl_baglanti.setStyleSheet(f"color: {KIRMIZI}; font-size: 9pt; background: transparent; font-weight: bold;")
        lay.addWidget(self.lbl_baglanti)

        # ROV sayacı
        self.lbl_rov_say = QLabel("ROV: –")
        self.lbl_rov_say.setStyleSheet(f"color: {METiN_KOYU}; font-size: 9pt; background: transparent; margin-left: 20px;")
        lay.addWidget(self.lbl_rov_say)

        # Düzen sıfırla butonu
        btn_duzen = QPushButton("⊞ Düzen Sıfırla")
        btn_duzen.setFixedHeight(28)
        btn_duzen.setStyleSheet(f"""
            QPushButton {{
                background: #1a2535; color: {METiN_KOYU};
                border: 1px solid #1e2d40; border-radius: 4px;
                padding: 0 10px; font-size: 8pt; margin-left: 16px;
            }}
            QPushButton:hover {{ color: {VURGU}; border-color: {VURGU}; }}
        """)
        btn_duzen.clicked.connect(self.duzen_sifirla)
        lay.addWidget(btn_duzen)

        # Kılavuz butonu
        btn_kilavuz = QPushButton("? Kılavuz")
        btn_kilavuz.setFixedHeight(28)
        btn_kilavuz.setStyleSheet(f"""
            QPushButton {{
                background: #1a2535; color: {METiN_KOYU};
                border: 1px solid #1e2d40; border-radius: 4px;
                padding: 0 10px; font-size: 8pt; margin-left: 6px;
            }}
            QPushButton:hover {{ color: {VURGU}; border-color: {VURGU}; }}
        """)
        btn_kilavuz.clicked.connect(self._kilavuz_ac)
        lay.addWidget(btn_kilavuz)

    def _kilavuz_ac(self):
        dlg = KilavuzDialog(self.window())
        dlg.exec_()

    def durum_guncelle(self, bagli: bool, rov_sayisi: int):
        if bagli:
            self.lbl_baglanti.setText("● BAĞLI")
            self.lbl_baglanti.setStyleSheet(f"color: {YESiL}; font-size: 9pt; background: transparent; font-weight: bold;")
        else:
            self.lbl_baglanti.setText("● BAĞLANTI YOK")
            self.lbl_baglanti.setStyleSheet(f"color: {KIRMIZI}; font-size: 9pt; background: transparent; font-weight: bold;")
        self.lbl_rov_say.setText(f"ROV: {rov_sayisi}")


# ── Sol Durum Çubuğu (ROV Göstergesi) ────────────────────────────────────────
class DurumCubugu(QStatusBar):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QStatusBar {{
                background-color: #080c11;
                color: {METiN_KOYU};
                border-top: 1px solid #1e2d40;
                font-size: 8pt;
            }}
            QStatusBar::item {{ border: none; }}
        """)
        self.setSizeGripEnabled(False)
        self.lbl_mesaj = QLabel("Hazır")
        self.lbl_mesaj.setStyleSheet(f"color: {METiN_KOYU};")
        self.addWidget(self.lbl_mesaj, 1)

    def mesaj_goster(self, metin: str, seviye: str = "ok"):
        renk = {
            "ok":   YESiL,
            "warn": SARI,
            "err":  KIRMIZI,
        }.get(seviye, METiN_KOYU)
        self.lbl_mesaj.setStyleSheet(f"color: {renk};")
        self.lbl_mesaj.setText(metin)


# ── Ana Pencere ───────────────────────────────────────────────────────────────
class KomutaMerkezi(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FiratROVNet — Komuta Merkezi")
        self.setMinimumSize(1280, 780)
        self.resize(1440, 860)
        self.setStyleSheet(GLOBAL_QSS)

        # Sinyal köprüsü
        self.sinyal = SinyalKoprusu()
        self.sinyal.durum_guncellendi.connect(self._on_durum)

        # ─ Merkezi widget
        merkez = QWidget()
        self.setCentralWidget(merkez)
        ana_lay = QVBoxLayout(merkez)
        ana_lay.setContentsMargins(0, 0, 0, 0)
        ana_lay.setSpacing(0)

        # ─ Başlık
        self.baslik = BaslikCubugu()
        ana_lay.addWidget(self.baslik)

        # ─ Gövde
        govde = QWidget()
        ana_lay.addWidget(govde, 1)
        govde_lay = QVBoxLayout(govde)
        govde_lay.setContentsMargins(8, 8, 8, 8)
        govde_lay.setSpacing(0)

        # Sol sütun: ROV + Grup
        sol_sutun = QWidget()
        sol_sutun.setMinimumWidth(200)
        sol_lay = QVBoxLayout(sol_sutun)
        sol_lay.setContentsMargins(0, 0, 0, 0)
        sol_lay.setSpacing(8)

        self.rov_panel = ROVPanel(sinyal=self.sinyal)
        sol_lay.addWidget(self.rov_panel, 1)

        # Orta sütun: Formasyon + Görev (sekme)
        orta_sutun = QWidget()
        orta_lay = QVBoxLayout(orta_sutun)
        orta_lay.setContentsMargins(0, 0, 0, 0)
        orta_lay.setSpacing(8)

        self.sekme = QTabWidget()
        self.sekme.setTabPosition(QTabWidget.North)

        self.gorev_panel     = GorevPanel(sinyal=self.sinyal,
                                          rov_panel_ref=self.rov_panel)
        self.surucu_panel    = SurucuPanel(sinyal=self.sinyal)

        self.sekme.addTab(self.surucu_panel,    "🐙  SÜRÜ")
        self.sekme.addTab(self.gorev_panel,     "🎯  GÖREV")

        orta_lay.addWidget(self.sekme, 1)

        # Sağ sütun: Komut çıkışı
        self.komut_panel = KomutPanel(sinyal=self.sinyal)
        self.komut_panel.setMinimumWidth(280)

        # ── Ana Splitter (yatay, sürükle-bırak boyutlandır) ──
        self._splitter = QSplitter(Qt.Horizontal)
        self._splitter.setHandleWidth(6)
        self._splitter.setStyleSheet("""
            QSplitter::handle { background: #1e2d40; }
            QSplitter::handle:hover { background: #00e5ff; }
        """)
        self._splitter.addWidget(sol_sutun)
        self._splitter.addWidget(orta_sutun)
        self._splitter.addWidget(self.komut_panel)
        self._splitter.setCollapsible(0, False)
        self._splitter.setCollapsible(1, False)
        self._splitter.setCollapsible(2, False)

        # Kaydedilmiş boyutları yükle
        _settings = QSettings("FiratROVNet", "KomutaMerkezi")
        _saved_state = _settings.value("splitter_state")
        if _saved_state:
            self._splitter.restoreState(_saved_state)
        else:
            self._splitter.setSizes([280, 800, 380])

        govde_lay.addWidget(self._splitter, 1)

        # ─ Alt durum çubuğu
        self.durum_cubugu = DurumCubugu()
        self.setStatusBar(self.durum_cubugu)

        # ─ Paneller arası bağlantılar
        self._panel_baglantilari()
        self.baslik.duzen_sifirla.connect(self._duzen_sifirla)

        # ─ Periyodik güncelleme (1 saniye)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._periyodik_guncelle)
        self._timer.start(1000)

        # İlk güncelleme
        self._periyodik_guncelle()

    # ── Paneller arası sinyal bağlantıları ───────────────────────────────────
    def _panel_baglantilari(self):
        self.gorev_panel.komut_uretildi.connect(self.komut_panel.komut_ekle)
        self.rov_panel.komut_uretildi.connect(self.komut_panel.komut_ekle)
        self.surucu_panel.komut_uretildi.connect(self.komut_panel.komut_ekle)

        # ROV listesi butonları → SÜRÜ panelinin doğru metodlarına delegate
        # (find_rov_by_id + group_id + cache mantığı surucu_panel'da merkezi)
        self.rov_panel.lider_talep.connect(self.surucu_panel._lider_olustur)
        self.rov_panel.takipci_talep.connect(self.surucu_panel._us_a_birak)

    # ── Periyodik güncelleme ──────────────────────────────────────────────────
    def _periyodik_guncelle(self):
        rovlar = rov_listesi()
        bagli  = sim_bagli_mi()
        self.baslik.durum_guncelle(bagli, len(rovlar))
        self.rov_panel.rov_listesini_guncelle(rovlar)
        self.surucu_panel.rov_listesini_guncelle(rovlar)

    # ── Durum mesajı ─────────────────────────────────────────────────────────
    def _on_durum(self, mesaj: str, seviye: str):
        self.durum_cubugu.mesaj_goster(mesaj, seviye)

    def _duzen_sifirla(self):
        """Splitter boyutlarını varsayılana döndür ve kaydı sil."""
        self._splitter.setSizes([280, 800, 380])
        QSettings("FiratROVNet", "KomutaMerkezi").remove("splitter_state")

    def closeEvent(self, e):
        """Pencere kapanırken splitter boyutlarını kaydet."""
        QSettings("FiratROVNet", "KomutaMerkezi").setValue(
            "splitter_state", self._splitter.saveState()
        )
        super().closeEvent(e)


# ── Giriş Noktası ────────────────────────────────────────────────────────────
def baslat(filo=None):
    """
    Arayüzü başlatır.
    filo nesnesi verilirse simülasyona bağlı çalışır.
    Kullanım:
        from UI.ana_pencere import baslat
        baslat(filo=filo)
    """
    if filo is not None:
        from UI.kopru import filo_bagla
        filo_bagla(filo)

    app = QApplication.instance() or QApplication(sys.argv)
    pencere = KomutaMerkezi()
    pencere.show()
    return pencere, app


if __name__ == "__main__":
    pencere, app = baslat()
    sys.exit(app.exec_())
