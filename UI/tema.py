"""
FiratROVNet Komuta Arayüzü — Askeri Tema Sabitleri
Tüm renk, font ve stil tanımları burada merkezi olarak yönetilir.
"""

# ── Renk Paleti ───────────────────────────────────────────────────────────────
ARKA_PLAN       = "#0a0d12"      # Ana arka plan (neredeyse siyah)
PANEL           = "#111720"      # Panel arka planı
PANEL_KENAR     = "#1e2d40"      # Panel kenar rengi
VURGU           = "#00e5ff"      # Cyan vurgu (aktif, seçili)
VURGU_KOYU      = "#0097a7"      # Cyan koyu ton
YESiL           = "#00c853"      # Başarı / aktif görev
SARI            = "#ffd600"      # Uyarı
KIRMIZI         = "#ff1744"      # Tehlike / durdur
TURUNCU         = "#ff6d00"      # Bekleyen / transit
METiN           = "#cfd8dc"      # Ana metin
METiN_KOYU      = "#546e7a"      # İkincil metin / etiket
BUTON_NORMAL    = "#1a2535"      # Buton arka plan
BUTON_HOVER     = "#1e3a5f"      # Buton hover
BUTON_AKTIF     = "#0d2137"      # Buton basılı
SECENEK_SEÇILI  = "#003049"      # Seçili satır/kart arka planı
GOREV_RENK      = {
    "alan_tarama":      "#00bcd4",
    "arama_kurtarma":   "#ffd600",
    "imha":             "#ff1744",
    "idle":             "#546e7a",
}

# ── Fontlar ───────────────────────────────────────────────────────────────────
FONT_MONO   = "Consolas"          # Komut çıkışı, log
FONT_UI     = "Segoe UI"          # Genel UI metni
FONT_TITLE  = "Segoe UI Semibold"
FONT_BOYUT  = {
    "baslik":   14,
    "normal":   10,
    "kucuk":    8,
    "buyuk":    16,
    "komut":    9,
}

# ── QSS (PyQt5 stylesheet) ────────────────────────────────────────────────────
GLOBAL_QSS = f"""
/* ─── Ana Pencere ─────────────────────────────────────────── */
QMainWindow, QDialog {{
    background-color: {ARKA_PLAN};
    color: {METiN};
}}
QWidget {{
    background-color: {ARKA_PLAN};
    color: {METiN};
    font-family: "{FONT_UI}";
    font-size: {FONT_BOYUT['normal']}pt;
}}

/* ─── Çerçeve / Grup ──────────────────────────────────────── */
QGroupBox {{
    background-color: {PANEL};
    border: 1px solid {PANEL_KENAR};
    border-radius: 6px;
    margin-top: 12px;
    padding-top: 8px;
    font-size: {FONT_BOYUT['normal']}pt;
    font-weight: bold;
    color: {VURGU};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
    color: {VURGU};
    font-size: {FONT_BOYUT['normal']}pt;
}}

/* ─── Butonlar ────────────────────────────────────────────── */
QPushButton {{
    background-color: {BUTON_NORMAL};
    color: {METiN};
    border: 1px solid {PANEL_KENAR};
    border-radius: 5px;
    padding: 6px 14px;
    font-size: {FONT_BOYUT['normal']}pt;
    min-height: 28px;
}}
QPushButton:hover {{
    background-color: {BUTON_HOVER};
    border: 1px solid {VURGU};
    color: {VURGU};
}}
QPushButton:pressed {{
    background-color: {BUTON_AKTIF};
    border: 1px solid {VURGU_KOYU};
}}
QPushButton:disabled {{
    background-color: {ARKA_PLAN};
    color: {METiN_KOYU};
    border: 1px solid #1a2535;
}}
QPushButton#btn_basla {{
    background-color: #003d1f;
    color: {YESiL};
    border: 1px solid {YESiL};
    font-weight: bold;
    font-size: 12pt;
    min-height: 38px;
}}
QPushButton#btn_basla:hover {{
    background-color: #005229;
}}
QPushButton#btn_durdur {{
    background-color: #3d0010;
    color: {KIRMIZI};
    border: 1px solid {KIRMIZI};
    font-weight: bold;
}}
QPushButton#btn_durdur:hover {{
    background-color: #5c0016;
}}
QPushButton#btn_formasyon_sec {{
    background-color: #1a1a00;
    color: {SARI};
    border: 1px solid {SARI};
}}

/* ─── ComboBox ────────────────────────────────────────────── */
QComboBox {{
    background-color: {BUTON_NORMAL};
    color: {METiN};
    border: 1px solid {PANEL_KENAR};
    border-radius: 4px;
    padding: 4px 8px;
    min-height: 26px;
}}
QComboBox:focus {{
    border: 1px solid {VURGU};
}}
QComboBox QAbstractItemView {{
    background-color: {PANEL};
    color: {METiN};
    selection-background-color: {SECENEK_SEÇILI};
    selection-color: {VURGU};
    border: 1px solid {PANEL_KENAR};
}}
QComboBox::drop-down {{
    border: none;
    width: 20px;
}}

/* ─── SpinBox / DoubleSpinBox ─────────────────────────────── */
QSpinBox, QDoubleSpinBox {{
    background-color: {BUTON_NORMAL};
    color: {METiN};
    border: 1px solid {PANEL_KENAR};
    border-radius: 4px;
    padding: 3px 6px;
    min-height: 26px;
}}
QSpinBox:focus, QDoubleSpinBox:focus {{
    border: 1px solid {VURGU};
}}

/* ─── ListView / ListWidget ───────────────────────────────── */
QListWidget {{
    background-color: {PANEL};
    color: {METiN};
    border: 1px solid {PANEL_KENAR};
    border-radius: 4px;
    outline: 0;
}}
QListWidget::item {{
    padding: 5px 8px;
    border-bottom: 1px solid #151d28;
}}
QListWidget::item:selected {{
    background-color: {SECENEK_SEÇILI};
    color: {VURGU};
    border-left: 3px solid {VURGU};
}}
QListWidget::item:hover {{
    background-color: #141e2e;
}}

/* ─── Tab Widget ──────────────────────────────────────────── */
QTabWidget::pane {{
    border: 1px solid {PANEL_KENAR};
    background-color: {PANEL};
    border-radius: 4px;
}}
QTabBar::tab {{
    background-color: {ARKA_PLAN};
    color: {METiN_KOYU};
    border: 1px solid {PANEL_KENAR};
    border-bottom: none;
    padding: 7px 18px;
    margin-right: 2px;
    border-radius: 4px 4px 0 0;
}}
QTabBar::tab:selected {{
    background-color: {PANEL};
    color: {VURGU};
    border-bottom: 2px solid {VURGU};
}}
QTabBar::tab:hover {{
    color: {METiN};
    background-color: {BUTON_HOVER};
}}

/* ─── Kaydırma Çubuğu ────────────────────────────────────── */
QScrollBar:vertical {{
    background: {ARKA_PLAN};
    width: 8px;
    border-radius: 4px;
}}
QScrollBar::handle:vertical {{
    background: {PANEL_KENAR};
    border-radius: 4px;
    min-height: 30px;
}}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
    height: 0;
}}

/* ─── TextEdit / PlainTextEdit ────────────────────────────── */
QTextEdit, QPlainTextEdit {{
    background-color: #080c11;
    color: #00e5cc;
    border: 1px solid {PANEL_KENAR};
    border-radius: 4px;
    font-family: "{FONT_MONO}";
    font-size: {FONT_BOYUT['komut']}pt;
}}

/* ─── Label ───────────────────────────────────────────────── */
QLabel {{
    color: {METiN};
    background: transparent;
}}
QLabel[role="baslik"] {{
    color: {VURGU};
    font-size: {FONT_BOYUT['baslik']}pt;
    font-weight: bold;
}}
QLabel[role="etiket"] {{
    color: {METiN_KOYU};
    font-size: {FONT_BOYUT['kucuk']}pt;
}}

/* ─── CheckBox ────────────────────────────────────────────── */
QCheckBox {{
    color: {METiN};
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid {PANEL_KENAR};
    border-radius: 3px;
    background: {BUTON_NORMAL};
}}
QCheckBox::indicator:checked {{
    background: {VURGU};
    border-color: {VURGU};
}}

/* ─── RadioButton ─────────────────────────────────────────── */
QRadioButton {{
    color: {METiN};
    spacing: 6px;
}}
QRadioButton::indicator {{
    width: 14px;
    height: 14px;
    border: 1px solid {PANEL_KENAR};
    border-radius: 7px;
    background: {BUTON_NORMAL};
}}
QRadioButton::indicator:checked {{
    background: {VURGU};
    border-color: {VURGU};
}}

/* ─── Slider ──────────────────────────────────────────────── */
QSlider::groove:horizontal {{
    height: 4px;
    background: {PANEL_KENAR};
    border-radius: 2px;
}}
QSlider::handle:horizontal {{
    background: {VURGU};
    border: 1px solid {VURGU_KOYU};
    width: 14px;
    height: 14px;
    border-radius: 7px;
    margin: -5px 0;
}}
QSlider::sub-page:horizontal {{
    background: {VURGU};
    border-radius: 2px;
}}

/* ─── Splitter ────────────────────────────────────────────── */
QSplitter::handle {{
    background: {PANEL_KENAR};
    width: 2px;
    height: 2px;
}}

/* ─── Scrollarea ──────────────────────────────────────────── */
QScrollArea {{
    border: none;
    background: transparent;
}}
"""
