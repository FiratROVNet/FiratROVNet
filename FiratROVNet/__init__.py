import torch
import os

# 1. Alt Modüllere Erişim (Opsiyonel ama yararlı)
from . import gat
from . import mappo
from . import ortam
from . import senaryo

# 2. Ana Sınıfları Doğrudan Dışarı Aktarma (Kullanım Kolaylığı İçin)
from .gat import GAT_Modeli, Train, FiratAnalizci 
from .ortam import veri_uret
from .senaryo import Senaryo, uret, get, set, git, guncelle, temizle

# 3. Kütüphane Bilgileri (Metadata)
__university__ = "Fırat Üniversitesi"
__lab__ = "Otonom Sistemler & Yapay Zeka Laboratuvarı"
__version__ = "1.7.3"
__author__ = "Ömer Faruk Çelik"

# 4. Dışarıdan Erişilebilecekler Listesi (Kritik Kısım)
# 'from FiratROVNet import *' dendiğinde bunlar gelir.
__all__ = [
    # Modüller
    'gat', 
    'mappo', 
    'ortam',
    'senaryo',
    
    # Sınıflar ve Fonksiyonlar
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
