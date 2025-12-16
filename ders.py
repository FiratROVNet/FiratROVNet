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

try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ Model yüklenemedi, AI devre dışı."); 
    beyin = None

print("Burada: ",len(app.rovs))

filo=Filo()
# ROV rollerini manuel olarak ayarla
app.rovs[1].set("rol", 1)  # ROV-1 lider
for i in range(len(app.rovs)):
    if i != 1:
        app.rovs[i].set("rol", 0)  # Diğerleri takipçi

modem = filo.otomatik_kurulum(app.rovs)


filo.manuel_kontrol_all(True)
	

rov_id=3
x=100
y=100
z=0


app.filo=filo

filo.set(rov_id,"rol",1)
filo.set(4,"rol",1)
filo.set(1,"rol",1)


#get ile alabileceğin sensörler verileri: batarya, sonar, hiz, gps
print("Batarya:",filo.get(2,"batarya"))
print("Sonar:",filo.get(4,"sonar"))
filo.move(0,"ileri",0.1)
#filo.git(rov_id,x,z,y,False)

#filo.move(rov_id,"ileri",0.1)

#batarya,gps,hiz,rol,sonar












# 2. ANA DÖNGÜ
def update():
    try:
    
       	#print("Batarya:",filo.get(2,"batarya"))  
       	print("GPS:",filo.get(0,"gps")) 
        veri = app.simden_veriye()
        
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
