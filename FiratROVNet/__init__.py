import torch
import os

# 1. Alt Modüllere Erişim (Opsiyonel ama yararlı)
# GAT modülü artık GAT/ klasöründe
try:
    from GAT.gat_train import GAT_Modeli, train as Train
    from GAT.gat_test import FiratAnalizci
    gat = None  # Eski modül artık yok
except ImportError:
    # GAT modülü yüklenemezse (torch_geometric yoksa) sessizce geç
    gat = None
    GAT_Modeli = None
    Train = None
    FiratAnalizci = None

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
__version__ = "1.7.7"
__author__ = "Ömer Faruk Çelik"

# 4. Dışarıdan Erişilebilecekler Listesi (Kritik Kısım)
# 'from FiratROVNet import *' dendiğinde bunlar gelir.
__all__ = [
    # Modüller (None olabilir)
    'senaryo',
    
    # Sınıflar ve Fonksiyonlar (None olabilir)
    'GAT_Modeli', 
    'Train', 
    'FiratAnalizci', 
    'Senaryo',
    'uret',
    'get',
    'set',
    'git',
    'guncelle',
    'temizle'
]
