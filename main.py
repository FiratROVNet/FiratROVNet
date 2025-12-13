from FiratROVNet.simulasyon import Ortam
from FiratROVNet.iletisim import AkustikModem
from FiratROVNet.gnc import GNCKomutan, LiderGNC
from ursina import *
import numpy as np
import sys

# 1. ORTAM KURULUMU
print("🌊 [ADIM 3] Zamanlı Görev Testi (DÜZELTİLMİŞ)")
app = Ortam()
app.sim_olustur(n_rovs=4, n_engels=10)


komuta = GNCKomutan()

# 2. KOMUTAN VE ROV KURULUMU (TOPLU KAYIT)
for i, rov in enumerate(app.rovs):
    # Her araca bir modem ve beyin takıyoruz
    modem = AkustikModem(rov_id=i)
    rov.modem = modem
    gnc = LiderGNC(rov, modem)
    
    # Komutana sırasıyla ekliyoruz (0, 1, 2, 3...)
    komuta.ekle(gnc) 
    




ROV2 = app.rovs[2]
ROV2.color = color.pink
baslangic_pos = ROV2.position #Başlangışta ROV'un bulunduğu konum bilgisini alır
# Hata almamak için TÜM ROV'ları sırasıyla sisteme ekliyoruz

# --- ZAMAN VE DURUM DEĞİŞKENLERİ ---
zaman_sayaci = 0.0
gorev_asamasi = 0  
# 0: Bekliyor (15 sn)
# 1: Hedefe Gidiyor
# 2: Geri Dönüyor
# 3: Görev Bitti

def update():
    global zaman_sayaci, gorev_asamasi
    
    # 1. Komutanı her karede güncelle (Motorları çalıştırır)
    bos_tahminler = np.zeros(len(app.rovs), dtype=int)
    komuta.guncelle_hepsi(bos_tahminler)
    
    # 2. Zamanı Say
    if gorev_asamasi < 3: 
        zaman_sayaci += time.dt

    # --- DURUM MAKİNESİ ---
    
    # AŞAMA 0: BEKLEME
    if gorev_asamasi == 0:
        print(f"\r⏳ Başlamaya kalan: {15 - zaman_sayaci:.1f} sn", end="")
        
        if zaman_sayaci >= 15.0:
            print("\n🚀 15 Saniye Doldu! ROV-2 Hedefe (20, -5, 30) gidiliyor...")
            
            # Artık ID:2 geçerli çünkü döngüde 0,1,2,3 hepsini ekledik.
            komuta.git(2, 20, 30, -5,False)
            gorev_asamasi = 1

    # AŞAMA 1: HEDEFE GİDİŞ KONTROLÜ
    elif gorev_asamasi == 1:
        hedef_vektor = Vec3(20, -5, 30)
        
        # Hedefe 2 birimden fazla yaklaştıysa varmış sayalım
        if distance(ROV2.position, hedef_vektor) < 6.0:
            print(f"\n✅ Hedefe Varıldı! Başlangıç noktasına ({baslangic_pos}) dönülüyor...")
            
            # Geri Dönüş Komutu
            komuta.git(2, baslangic_pos.x, baslangic_pos.z, baslangic_pos.y,False)
            gorev_asamasi = 2

    # AŞAMA 2: DÖNÜŞ KONTROLÜ
    elif gorev_asamasi == 2:
        if distance(ROV2.position, baslangic_pos) < 2.0:
            print("\n🎉 Başlangıç noktasına geri dönüldü. GÖREV TAMAMLANDI.")
            gorev_asamasi = 3








def input(key):
    if key == 'q' or key == 'escape':
        sys.exit()

app.set_update_function(update)
app.app.input = input

app.run()
