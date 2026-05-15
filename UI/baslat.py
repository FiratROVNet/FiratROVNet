"""
FiratROVNet Komuta Merkezi — Başlatıcı
Kullanım:
  python UI/baslat.py                    # Bağımsız (simülasyonsuz)
  python UI/baslat.py --sim              # main.py'yi başlatır (simülasyonla birlikte)
"""

import sys
import os

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# OpenCV, import edilince QT_QPA_PLATFORM_PLUGIN_PATH'i kendi cv2/qt/plugins
# dizinine ayarlayabiliyor. PyQt5 conda ortamında farklı bir plugin dizini
# kullanır; doğru dizini Qt'nin kendisinden okuyup cv2 yolunu eziyoruz.
try:
    from PyQt5.QtCore import QLibraryInfo
    _qt_plugins = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    if os.path.isdir(_qt_plugins):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _qt_plugins
        os.environ["QT_PLUGIN_PATH"] = _qt_plugins
except Exception:
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    os.environ.pop("QT_PLUGIN_PATH", None)

from PyQt5.QtWidgets import QApplication
from UI.ana_pencere import baslat

if __name__ == "__main__":
    pencere, app = baslat(filo=None)
    sys.exit(app.exec_())
