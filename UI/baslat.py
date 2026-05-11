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

# OpenCV'nin Qt eklentileri ile çakışmayı önle:
# cv2, kendi Qt5 eklentilerini QT_QPA_PLATFORM_PLUGIN_PATH'e ekleyebiliyor;
# bunu PyQt5'in diziniyle eziyoruz, böylece doğru 'xcb' eklentisi yüklenir.
try:
    import PyQt5 as _pyqt5
    _pyqt5_dir = os.path.dirname(_pyqt5.__file__)
    _qt_plugins = os.path.join(_pyqt5_dir, "Qt5", "plugins")
    if os.path.isdir(_qt_plugins):
        os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = _qt_plugins
except Exception:
    pass

from PyQt5.QtWidgets import QApplication
from UI.ana_pencere import baslat

if __name__ == "__main__":
    pencere, app = baslat(filo=None)
    sys.exit(app.exec_())
