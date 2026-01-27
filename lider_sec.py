from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.gat import FiratAnalizci
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import torch
import os
 
 #kod satır aralığı başlangıç
  
  
app=Ortam()
app.sim_olustur(6,25)
rovs=app.rovs
filo=Filo()
modem=filo.otomatik_kurulum(rovs,3)
# Filo referansını Ortam'a ekle
app.filo = filo


try: 
    beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")
except: 
    print("⚠️ Model yüklenemedi, AI devre dışı."); 
    beyin = None
#filo.set(rov_id,"rol",1) rov_id li rovu lider yapar, 0 ise takipci yapar

  
  
  # Kod başlangıcı

import math

# --- 1. LİDER SEÇİM SINIFI (Algoritma) ---
class LiderSecimModulu:
    def _init_(self):
        pass

    def mesafe_hesapla(self, pos1, pos2):
        """İki nokta arası Öklid mesafesi"""
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def a_star_simulasyonu(self, baslangic, hedef):
        """
        Simülasyon amaçlı A* mesafesi (Kuş uçuşu * 1.2)
        """
        kus_bakisi = self.mesafe_hesapla(baslangic, hedef)
        return kus_bakisi * 1.2 

    def deger_duzenle(self, deger):
        """
        KURAL: Değer 1'den küçükse 1'e yuvarla. Değilse olduğu gibi bırak.
        Bölenin 0 olmasını veya sonucun aşırı büyümesini engeller.
        """
        if deger < 1:
            return 1.0
        return deger

    def lideri_belirle_ve_yazdir(self, rov_listesi, hedef_konum):
        lider_skorlari = []
        detayli_sonuclar = []
        
        # --- P4 İÇİN MERKEZ HESABI ---
        merkez_uzakliklari = []
        for i in range(len(rov_listesi)):
            toplam_mesafe = 0
            for j in range(len(rov_listesi)):
                if i == j: continue 
                dist = self.mesafe_hesapla(rov_listesi[i]['konum'], rov_listesi[j]['konum'])
                toplam_mesafe += dist
            merkez_uzakliklari.append(toplam_mesafe)

        # --- HESAPLAMA DÖNGÜSÜ ---
        for i, rov in enumerate(rov_listesi):
            # P1: Batarya (0-1 arası normalize edilir)
            p1 = rov['batarya'] / 100.0
            
            # P2: Derinlik (Mutlak değer, Min 1)
            ham_derinlik = abs(rov['konum'][2]) 
            p2 = self.deger_duzenle(ham_derinlik)
            
            # P3: Hedef Mesafe (Min 1)
            ham_mesafe = self.a_star_simulasyonu(rov['konum'], hedef_konum)
            p3 = self.deger_duzenle(ham_mesafe)
            
            # P4: Merkezilik (Min 1)
            ham_merkez = merkez_uzakliklari[i]
            p4 = self.deger_duzenle(ham_merkez)
            
            # FORMÜL: P1 / (P2 * P3 * P4)
            # Batarya yüksek olsun; derinlik, mesafe ve merkeze uzaklık az olsun.
            payda = p2 * p3 * p4
            skor = p1 / payda
            
            lider_skorlari.append(skor)
            
            detayli_sonuclar.append({
                'id': rov['id'],
                'p1': p1,
                'p2': p2,
                'p3': p3,
                'p4': p4,
                'skor': skor
            })

        # En yüksek skoru ve lideri bul
        if not lider_skorlari:
            print("HATA: ROV listesi boş!")
            return -1, 0

        max_skor = max(lider_skorlari)
        lider_index = lider_skorlari.index(max_skor)
        secilen_rov_id = rov_listesi[lider_index]['id']
        
        #print(f" >>> SEÇİLEN LİDER: ROV #{secilen_rov_id} (Skor: {max_skor:.8f})")
        #print("="*85 + "\n")
        
        return secilen_rov_id, max_skor

# --- 2. ENTEGRASYON VE ÇALIŞTIRMA KISMI ---

def liderlik_secimini_baslat(filo_nesnesi,hedef_konum):
    """
    Bu fonksiyonu ana döngünüzün (main loop) içinde çağırabilirsiniz.
    """
    
    # A. Hazırlık
    rovlar_listesi = []
    # Hedef Konum: [x, y, z]. Z ekseni derinliktir.
    
    
    # B. Dinamik Veri Toplama (Sizin yazdığınız kısım)
    # range(len(...)) kullanarak 0'dan başlayıp tüm araçları geziyoruz.
    try:
        sistem_sayisi = len(filo_nesnesi.sistemler)
        
        for rid in range(sistem_sayisi):
            # Batarya 0-1 arasındaysa 100 ile çarpıp 0-100 formatına getiriyoruz
            bat = filo_nesnesi.get(rid, "batarya") * 100 
            
            # GPS verisi [x, y, z] döner
            gps = filo_nesnesi.get(rid, "gps")
            
            # Listeyi oluşturuyoruz
            rovlar_listesi.append({
                'id': rid,
                'batarya': bat,
                'konum': gps
            })
            
    except Exception as e:
        print(f"Veri çekme sırasında hata oluştu: {e}")
        return

    # C. Lider Seçim Modülünü Çalıştırma
    lider_modulu = LiderSecimModulu()
    secilen_id, skor = lider_modulu.lideri_belirle_ve_yazdir(rovlar_listesi, hedef_konum)
    
    return secilen_id,skor

# --- 3. KULLANIM ÖRNEĞİ ---
# Not: Bu kısım 'filo' nesneniz kodda tanımlıysa çalışacaktır.
# Eğer kodu bir fonksiyon içinde kullanacaksanız sadece yukarıdaki class'ı ve 
# liderlik_secimini_baslat fonksiyonunu almanız yeterli.

# Örnek Kullanım:
# lider_id = liderlik_secimini_baslat(filo)
# print(f"Ana kodda kullanılacak lider ID: {lider_id}")



# kod bitis
  
  
  
  
  
  
  
  
  

app.konsola_ekle("filo",filo)
def takipci_yap(lider_olacak):
    for i in range(len(filo.sistemler)):
        if i != lider_olacak:
            filo.set(i,"rol",0)
            x,y,z=filo.get(i,"gps")
            filo.git(i,x,y,-10)
            
def lider_kim():
    for i in range(len(filo.sistemler)):
        rol=filo.get(i,"rol")
        if rol==1:
            return i
# 2. ANA DÖNGÜ
def update():
    try:
        
        
        lider_id,skor=liderlik_secimini_baslat(filo,filo.asil_hedef)
        onceki_lider=lider_kim()
        
        
        if lider_id != onceki_lider:
            
            filo.set(lider_id,"rol",1)
            takipci_yap(lider_id)
            
        
        
        
        # Thread-safe komut kuyruğunu işle (konsoldan gelen komutlar için)
        filo.execute_queued_commands()
        
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
            # GAT kodunu ROV'a kaydet
            app.rovs[i].gat_kodu = gat_kodu
            
            if app.rovs[i].role == 1: 
                app.rovs[i].color = color.red
            else: 
                # GAT koduna göre renk değiştir (FBX model için de çalışır)
                app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
            
            # FBX model kullanılıyorsa, GAT kodunu görünür kılmak için color'ı blend et
            # (FBX model texture kullanıyorsa color değişimi daha az görünür olabilir)
            if hasattr(app.rovs[i], 'model') and isinstance(app.rovs[i].model, str) and app.rovs[i].model.endswith('.fbx'):
                # FBX model için color'ı daha belirgin yapmak için alpha veya tint kullan
                # Ursina'da color direkt olarak texture ile blend edilir
                pass  # Color zaten ayarlandı, Ursina otomatik blend eder
            
            # Label scale'ini büyüt (uzaktan okunabilir) - GAT kodu için daha büyük
            app.rovs[i].label.scale = 6000  # Sabit büyük scale
            app.rovs[i].label.y = 300  # Y eksenini artır (ROV'un üstünde daha yüksekte)
            app.rovs[i].label.color = app.rovs[i].color 
            app.rovs[i].label.background = False  # Arka plan ekle (daha görünür)
            
            ek = "" if ai_aktif else "\n[AI OFF]"
            # GAT kodunu label'da büyük ve görünür şekilde göster
            # gat_kodu bir integer, liste indexi olarak kullanılmalı
            gat_kodu = app.rovs[i].gat_kodu
            if 0 <= gat_kodu < len(durum_txts):
                app.rovs[i].label.text = durum_txts[gat_kodu]+str(i)
            else:
                app.rovs[i].label.text = f"GAT:{gat_kodu}+{str(i)}"
        
        filo.guncelle_hepsi(tahminler)
        
        # Harita güncelle (Matplotlib penceresi) - Throttled içeride yapılıyor
        if hasattr(app, 'harita') and app.harita is not None:
            try:
                # Matplotlib penceresini güncelle (throttled, non-blocking)
                app.harita.update()
                # plt.pause() kaldırıldı - harita.update() içinde throttle var
            except Exception as e:
                # Harita güncelleme hatası (sessizce geç, simülasyon devam etsin)
                pass
        
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
