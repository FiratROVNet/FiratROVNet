"""
Komut Paneli — Üretilen komutları önizler, geçmişi tutar ve uygular.
"""
from __future__ import annotations
import datetime

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QLabel,
    QPushButton, QPlainTextEdit, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, pyqtSignal, QMetaObject, Q_ARG
from PyQt5.QtGui import QFont, QColor, QTextCharFormat, QTextCursor

from UI.tema import (
    VURGU, METiN, METiN_KOYU, YESiL, KIRMIZI, SARI, PANEL, PANEL_KENAR,
    FONT_MONO,
)
from UI.kopru import komut_gonder


class KomutPanel(QWidget):
    # Thread-safe log sinyali: (html_metin)
    _log_ekle_sig    = pyqtSignal(str, str)   # renk, metin
    senkronize_talep = pyqtSignal()            # Sürü panelini UI ↔ Sim senkronize eder

    def __init__(self, sinyal=None, parent=None):
        super().__init__(parent)
        self.sinyal = sinyal
        self._gecmis: list[str] = []
        # Thread-safe log: arka thread'den gelen metinleri ana thread'de yazar
        self._log_ekle_sig.connect(self._log_satir_yaz)

        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        # ── Komut Geçmişi (Salt Okunur Log) ──────────────────────────────────
        gecmis_kutu = QGroupBox("KOMUT GEÇMİŞİ")
        gecmis_lay  = QVBoxLayout(gecmis_kutu)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setFont(QFont(FONT_MONO, 8))
        self.log.setMinimumHeight(200)
        self.log.setPlaceholderText("Henüz komut üretilmedi...")
        gecmis_lay.addWidget(self.log)

        # Log temizle
        btn_temizle = QPushButton("Geçmişi Temizle")
        btn_temizle.setFixedHeight(24)
        btn_temizle.clicked.connect(self.log.clear)
        gecmis_lay.addWidget(btn_temizle)

        lay.addWidget(gecmis_kutu, 3)

        # ── Manuel Komut Girişi ───────────────────────────────────────────────
        manuel_kutu = QGroupBox("MANUEL KOMUT")
        manuel_lay  = QVBoxLayout(manuel_kutu)

        self.txt_komut = QPlainTextEdit()
        self.txt_komut.setFont(QFont(FONT_MONO, 8))
        self.txt_komut.setFixedHeight(80)
        self.txt_komut.setPlaceholderText("filo.git(0, 100, 50, -20)")
        manuel_lay.addWidget(self.txt_komut)

        btn_lay = QHBoxLayout()
        self.btn_gonder = QPushButton("▶  Çalıştır")
        self.btn_gonder.setObjectName("btn_basla")
        self.btn_gonder.clicked.connect(self._manuel_gonder)
        btn_temizle_m = QPushButton("Temizle")
        btn_temizle_m.clicked.connect(self.txt_komut.clear)
        btn_lay.addWidget(self.btn_gonder)
        btn_lay.addWidget(btn_temizle_m)
        manuel_lay.addLayout(btn_lay)
        lay.addWidget(manuel_kutu, 1)

        # ── Hızlı Komutlar ────────────────────────────────────────────────────
        hizli_kutu = QGroupBox("HIZLI KOMUTLAR")
        hizli_lay  = QVBoxLayout(hizli_kutu)

        hizli_komutlar = [
            ("Tüm ROV'ları Durdur", "[filo.move(r.id, 'dur', 1.0) for r in filo.rovs if r]", "warn"),
            ("APF Temizle",         "filo.apf_temizle()",          "warn"),
        ]
        for metin, komut, seviye in hizli_komutlar:
            btn = QPushButton(metin)
            btn.setFixedHeight(26)
            btn.clicked.connect(
                lambda checked, k=komut, s=seviye: self._hizli_calistir(k, s)
            )
            hizli_lay.addWidget(btn)

        btn_sync = QPushButton("🔄  Sürü Panelini Senkronize Et")
        btn_sync.setFixedHeight(30)
        btn_sync.setToolTip("UI panel yapısını simülasyondaki mevcut ROV rol/grup değerlerine göre yeniden inşa eder")
        btn_sync.clicked.connect(self.senkronize_talep.emit)
        hizli_lay.addWidget(btn_sync)

        lay.addWidget(hizli_kutu)

    # ── Komut ekleme (panellerden sinyal) ─────────────────────────────────────
    def komut_ekle(self, komut: str, aciklama: str):
        zaman = datetime.datetime.now().strftime("%H:%M:%S")
        self._gecmis.append(komut)
        self._log_ekle_sig.emit(METiN_KOYU, f"[{zaman}] ")
        self._log_ekle_sig.emit(SARI,       f"{aciklama}\n")
        self._log_ekle_sig.emit(VURGU,      f"  {komut}\n\n")
        # Manuel alana kopyala
        self.txt_komut.setPlainText(komut)

    def _log_satir_yaz(self, renk: str, metin: str):
        """Ana thread'de QTextCursor ile log'a yazar."""
        cur = self.log.textCursor()
        cur.movePosition(QTextCursor.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(renk))
        cur.setCharFormat(fmt)
        cur.insertText(metin)
        self.log.setTextCursor(cur)
        self.log.ensureCursorVisible()

    def _manuel_gonder(self):
        komut = self.txt_komut.toPlainText().strip()
        if not komut:
            return
        komut_gonder(
            komut,
            callback=lambda s: self._sonuc_logla(komut, s),
        )

    def _hizli_calistir(self, komut: str, seviye: str):
        komut_gonder(
            komut,
            callback=lambda s: self._sonuc_logla(komut, s),
        )
        self.komut_ekle(komut, "Hızlı Komut")

    def _sonuc_logla(self, komut: str, sonuc: str):
        renk = YESiL if sonuc.startswith("✔") else (KIRMIZI if ("HATA" in sonuc or "✗" in sonuc) else SARI)
        # Sinyal üzerinden ana thread'e gönder (thread-safe)
        self._log_ekle_sig.emit(renk, f"  ↳ {sonuc}\n")
