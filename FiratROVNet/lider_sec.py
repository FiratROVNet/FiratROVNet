import math

class LiderSecimModulu:
    """
    Çoklu ROV sistemlerinde lider seçimi için kullanılan algoritma modülü.
    Batarya, derinlik, hedef mesafesi ve merkeziyet gibi parametreleri değerlendirir.
    """
    def __init__(self):
        pass

    def mesafe_hesapla(self, pos1, pos2):
        """İki nokta arası Öklid mesafesi hesaplar."""
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def a_star_simulasyonu(self, baslangic, hedef):
        """
        Simülasyon amaçlı A* mesafesi tahmini (Kuş uçuşu * 1.2).
        """
        kus_bakisi = self.mesafe_hesapla(baslangic, hedef)
        return kus_bakisi * 1.2 

    def deger_duzenle(self, deger):
        """
        Bölenin 0 olmasını veya sonucun aşırı büyümesini engellemek için
        değeri minimum 1.0 olarak ayarlar.
        """
        if deger < 1:
            return 1.0
        return deger

    def lideri_belirle(self, rov_listesi, hedef_konum):
        """
        Verilen ROV listesi ve hedef konuma göre en uygun lideri belirler.
        
        Args:
            rov_listesi (list): Her biri {'id': int, 'batarya': float, 'konum': [x,y,z]} içeren sözlük listesi.
            hedef_konum (list/tuple): Hedefin [x, y, z] koordinatları.
            
        Returns:
            tuple: (secilen_rov_id, max_skor)
        """
        lider_skorlari = []
        
        # --- P4: MERKEZİLİK HESABI ---
        merkez_uzakliklari = []
        for i in range(len(rov_listesi)):
            toplam_mesafe = 0
            for j in range(len(rov_listesi)):
                if i == j: continue 
                dist = self.mesafe_hesapla(rov_listesi[i]['konum'], rov_listesi[j]['konum'])
                toplam_mesafe += dist
            merkez_uzakliklari.append(toplam_mesafe)

        # --- SKOR HESAPLAMA DÖNGÜSÜ ---
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

        # En yüksek skoru ve lideri bul
        if not lider_skorlari:
            return -1, 0

        max_skor = max(lider_skorlari)
        lider_index = lider_skorlari.index(max_skor)
        secilen_rov_id = rov_listesi[lider_index]['id']
        
        return secilen_rov_id, max_skor

def liderlik_secimini_baslat(filo_nesnesi, hedef_konum):
    """
    Filo nesnesinden verileri çekip lider seçimini başlatan yardımcı fonksiyon.
    
    Args:
        filo_nesnesi: 'get(id, tip)' metoduna ve 'sistemler' listesine sahip filo objesi.
        hedef_konum: [x, y, z] formatında hedef koordinat.
        
    Returns:
        tuple: (secilen_id, skor)
    """
    rovlar_listesi = []
    
    try:
        # Filo yapısına göre sistem sayısını al (sistemler listesi veya rovs listesi olabilir)
        if hasattr(filo_nesnesi, 'sistemler'):
            sistem_sayisi = len(filo_nesnesi.sistemler)
        elif hasattr(filo_nesnesi, 'rovs'): # Alternatif yapı desteği
             sistem_sayisi = len(filo_nesnesi.rovs)
        else:
             print("Hata: Filo nesnesinde 'sistemler' veya 'rovs' bulunamadı.")
             return -1, 0
        
        for rid in range(sistem_sayisi):
            # Batarya verisi (0-1 arasındaysa 100 ile çarpıp 0-100 formatına getiriyoruz)
            bat_raw = filo_nesnesi.get(rid, "batarya")
            bat = bat_raw * 100 if bat_raw <= 1.0 else bat_raw
            
            # GPS verisi [x, y, z] döner
            gps = filo_nesnesi.get(rid, "gps")
            
            # Listeyi oluşturuyoruz
            rovlar_listesi.append({
                'id': rid,
                'batarya': bat,
                'konum': gps
            })
            
    except Exception as e:
        print(f"Lider seçiminde veri çekme hatası: {e}")
        return -1, 0

    # Modülü çalıştır
    modul = LiderSecimModulu()
    return modul.lideri_belirle(rovlar_listesi, hedef_konum)