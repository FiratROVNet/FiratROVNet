import os
os.environ["RR_IP_ADRESI"] = "127.0.0.1"

from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.config import cfg
from ursina import *
import cProfile
import pstats
import io
import time
import numpy as np

print("Ortam yükleniyor...")
app = Ortam()
app.sim_olustur(n_rovs=(7,), n_islands=4, havuz_genisligi=200, rov_model='submarine')
filo = Filo(ortam_ref=app)

# We need an update loop like in main.py
def custom_update():
    gps = filo.get(0, "gps") or Vec3(0,0,0)
    batarya = filo.get(0, "batarya") or 0
    tahminler = np.zeros(len(app.rovs), dtype=int)
    filo.guncelle_gat_analizi(tahminler)
    filo.guncelle_hepsi(tahminler, guncelle_gorseller=True)

app.set_update_function(custom_update)

print("Hazırlık tamam, 10 frame çalıştırılıyor ısınma için...")
for _ in range(10):
    app.app.step()

print("Profil başlatılıyor...")
pr = cProfile.Profile()
pr.enable()

for _ in range(100):
    app.app.step()

pr.disable()
print("Profil tamamlandı.")
ps = pstats.Stats(pr).sort_stats('cumtime')
ps.print_stats(40)
