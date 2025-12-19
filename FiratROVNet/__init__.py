import torch
import os

# 1. Alt Modüllere Erişim (Opsiyonel ama yararlı)
try:
    from . import gat
    from .gat import GAT_Modeli, Train, FiratAnalizci
except ImportError:
    # GAT modülü yüklenemezse (torch_geometric yoksa) sessizce geç
    gat = None
    GAT_Modeli = None
    Train = None
    FiratAnalizci = None

try:
    from . import mappo
except ImportError:
    mappo = None

try:
    from . import ortam
    from .ortam import veri_uret
except ImportError:
    ortam = None
    veri_uret = None

# Senaryo modülü (headless mod için önemli)
try:
    from . import senaryo
    from .senaryo import Senaryo, uret, get, set, git, guncelle, temizle
except ImportError as e:
    senaryo = None
    Senaryo = None
    uret = None
    get = None
    set = None
    git = None
    guncelle = None
    temizle = None

# 3. Kütüphane Bilgileri (Metadata)
__university__ = "Fırat Üniversitesi"
__lab__ = "Otonom Sistemler & Yapay Zeka Laboratuvarı"
__version__ = "1.7.5"
__author__ = "Ömer Faruk Çelik"

# 4. Dışarıdan Erişilebilecekler Listesi (Kritik Kısım)
# 'from FiratROVNet import *' dendiğinde bunlar gelir.
__all__ = [
    # Modüller (None olabilir)
    'gat', 
    'mappo', 
    'ortam',
    'senaryo',
    
    # Sınıflar ve Fonksiyonlar (None olabilir)
    'GAT_Modeli', 
    'Train', 
    'FiratAnalizci', 
    'veri_uret',
    'Senaryo',
    'uret',
    'get',
    'set',
    'git',
    'guncelle',
    'temizle'
]
