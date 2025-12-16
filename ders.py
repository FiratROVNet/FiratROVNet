from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import torch
import os

# Kod başlangıcı


app=Ortam()
app.sim_olustur(6,20)



print("Burada: ",len(app.rovs))

filo=Filo()
modem=filo.otomatik_kurulum(app.rovs,1)


filo.manuel_kontrol_all(True)
	

rov_id=2
x=-100
y=-100
z=0

filo.set(rov_id,"rol",1)
filo.git(rov_id,x,z,y,False)

filo.move(rov_id,"ileri",0.1)

#batarya,gps,hiz,rol,sonar
























# Kod bitişi
def simden_veriye():
    """Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)"""
    rovs = app.rovs
    engeller = app.engeller
    n = len(rovs)
    x = torch.zeros((n, 7), dtype=torch.float)
    positions = [r.position for r in rovs]
    sources, targets = [], []

    L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}

    for i in range(n):
        code = 0
        if i != 0 and distance(positions[i], positions[0]) > L['LEADER']: 
            code = 5
        dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
        if dists and min(dists) > L['DISCONNECT']: 
            code = 3
        
        min_engel = 999
        for engel in engeller:
            d = distance(positions[i], engel.position) - 6 
            if d < min_engel: 
                min_engel = d
        if min_engel < L['OBSTACLE']: 
            code = 1
        
        for j in range(n):
            if i != j and distance(positions[i], positions[j]) < L['COLLISION']:
                code = 2
                break
        
        x[i][0] = code / 5.0
        x[i][1] = rovs[i].battery / 100.0
        x[i][2] = 0.9
        x[i][3] = abs(rovs[i].y) / 100.0
        x[i][4] = rovs[i].velocity.x
        x[i][5] = rovs[i].velocity.z
        x[i][6] = rovs[i].role

        for j in range(n):
            if i != j and distance(positions[i], positions[j]) < L['DISCONNECT']:
                sources.append(i)
                targets.append(j)

    edge_index = torch.tensor([sources, targets], dtype=torch.long)
    class MiniData:
        def __init__(self, x, edge_index): 
            self.x, self.edge_index = x, edge_index
    return MiniData(x, edge_index)

# 3. ANA DÖNGÜ
def update():
    try:
        veri = simden_veriye()
        #print("gps:",filo.get(rov_id,"gps"))
        #print("Batarya: ",filo.get(rov_id,"batarya"))
        #print("sonar:",filo.get(3,"sonar"))
        print("HIZ:",filo.get(1,"hiz"))
        
        ai_aktif = getattr(cfg, 'ai_aktif', True)
        if ai_aktif and beyin:
            try: 
                tahminler, _, _ = beyin.analiz_et(veri)
            except: 
                tahminler = np.zeros(len(app.rovs), dtype=int)
        else:
            tahminler = np.zeros(len(app.rovs), dtype=int)

        kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
        durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
        
        for i, gat_kodu in enumerate(tahminler):
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}{ek}"
        
        filo.guncelle_hepsi(tahminler)
        
    except Exception as e: 
        pass

app.set_update_function(update)
# Input handler override edilmedi - Ursina'nın varsayılan input handler'ı çalışıyor
# EditorCamera'nın P tuşu ve diğer kontrolleri çalışacak

# 4. ÇALIŞTIRMA
if __name__ == "__main__":
    try: 
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        pass
    finally: 
        os.system('stty sane')
        os._exit(0)
