import torch
import os

# 1. Alt Modüllere Erişim (Opsiyonel ama yararlı)
from . import gat
from . import mappo
from . import ortam
from . import yonetici

# 2. Ana Sınıfları Doğrudan Dışarı Aktarma (Kullanım Kolaylığı İçin)
from .gat import GAT_Modeli, Train, FiratAnalizci 
from .ortam import veri_uret
from .yonetici import ROV_Sistemi

# 3. Kütüphane Bilgileri (Metadata)
__university__ = "Fırat Üniversitesi"
__lab__ = "Otonom Sistemler & Yapay Zeka Laboratuvarı"
__version__ = "1.5.0"
__author__ = "Ömer Faruk Çelik"

# 4. Dışarıdan Erişilebilecekler Listesi (Kritik Kısım)
# 'from FiratROVNet import *' dendiğinde bunlar gelir.
__all__ = [
    # Modüller
    'gat', 
    'mappo', 
    'ortam',
    
    # Sınıflar ve Fonksiyonlar
    'GAT_Modeli', 
    'Train', 
    'FiratAnalizci', 
    'veri_uret', 
    'ROV_Sistemi'
]
