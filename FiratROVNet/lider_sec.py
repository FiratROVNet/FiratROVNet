import math
import random
import numpy as np

class LiderSecimModulu:
    def __init__(self, filo_ref):
        self.filo_ref = filo_ref

    def mesafe_hesapla(self, pos1, pos2):
        if pos1 is None or pos2 is None: return 999.0
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2 + (pos1[2]-pos2[2])**2)

    def a_star_simulasyonu(self, baslangic, hedef):
        return self.mesafe_hesapla(baslangic, hedef) * 1.2 

    def deger_duzenle(self, deger):
        return max(1.0, float(deger))

    def lideri_belirle(self, rov_listesi_sozluk, hedef_konum):
        """
        Sözlük yapısına uyumlu lider belirleme.
        Girdi: {g_id: [{'id':.., 'batarya':.., 'konum':..}, ...]}
        Çıktı: {g_id: secilen_id}, {g_id: skor}
        """
        lider_skorlari = {}
        secilen_rov_id = {}

        # Sözlük üzerinde güvenli iterasyon
        for g_id, rov_listesi in rov_listesi_sozluk.items():
            # Grup boşsa atla
            if not rov_listesi:
                secilen_rov_id[g_id] = -1
                lider_skorlari[g_id] = 0
                continue

            # --- DURUM A: HEDEF YOKSA (Mevcut lideri koru veya random seç) ---
            if hedef_konum is None:
                lider_id, _ = self.filo_ref.find_leader_info(sessiz=True, g_id=g_id)
                #print(g_id,lider_id)
                
                if lider_id is None:
                    # Rastgele birinin ID'sini al
                    lider_id = random.choice(rov_listesi)["id"]
                    print(f"🎲 Grup-{g_id} için rastgele lider atandı.")

                secilen_rov_id[g_id] = lider_id
                lider_skorlari[g_id] = 1.0
                continue

            # --- DURUM B: HEDEF VARSA (Skor hesapla) ---
            max_lider_skor = -1.0
            en_uygun_id = rov_listesi[0]['id']

            # 1. MERKEZİLİK HESABI
            merkez_uzakliklari = []
            for i in range(len(rov_listesi)):
                toplam_mesafe = 0
                for j in range(len(rov_listesi)):
                    if i == j: continue 
                    toplam_mesafe += self.mesafe_hesapla(rov_listesi[i]['konum'], rov_listesi[j]['konum'])
                merkez_uzakliklari.append(toplam_mesafe)

            # 2. SKORLAMA DÖNGÜSÜ
            for i, rov in enumerate(rov_listesi):
                try:
                    p1 = rov['batarya'] / 100.0
                    p2 = self.deger_duzenle(abs(rov['konum'][2])) 
                    p3 = self.deger_duzenle(self.a_star_simulasyonu(rov['konum'], hedef_konum))
                    p4 = self.deger_duzenle(merkez_uzakliklari[i])
                    
                    # Formül: Batarya / (Derinlik * HedefMesafe * Merkezilik)
                    skor = p1 / (p2 * p3 * p4)

                    if skor > max_lider_skor:
                        max_lider_skor = skor
                        en_uygun_id = rov['id']
                except:
                    continue

            secilen_rov_id[g_id] = en_uygun_id
            lider_skorlari[g_id] = max_lider_skor

        return secilen_rov_id, lider_skorlari

def liderlik_secimini_baslat(filo_nesnesi, hedef_konum):
    """
    Sözlük tabanlı g_rovs yapısına tam uyumlu başlatıcı.
    NumPy array hatalarına karşı korumalıdır.
    """
    rovlar_data_sozlugu = {}

    try:
        # filo_nesnesi.g_rovs bir sözlük: {g_id: [Entity, Entity...]}
        for g_id, rov_grubu in filo_nesnesi.g_rovs.items():
            rovlar_data_sozlugu[g_id] = []

            for rov in rov_grubu:
                if not rov: continue

                # --- HATA DÜZELTME: GPS VERİSİNİ GÜVENLİ ÇEK ---
                gps = rov.get("gps")
                if gps is None:
                    gps = [0.0, 0.0, 0.0]
                
                # --- BATARYA VERİSİNİ GÜVENLİ ÇEK ---
                bat_raw = rov.get("batarya")
                if bat_raw is None:
                    bat_raw = 0.0
                
                # Batarya normalizasyonu (0-1 arasındaysa 100'e tamamla)
                bat = bat_raw * 100.0 if bat_raw <= 1.0 else bat_raw
                
                rovlar_data_sozlugu[g_id].append({
                    'id': rov.id,
                    'batarya': bat,
                    'konum': gps
                })
                
    except Exception as e:
        if hasattr(filo_nesnesi, 'ds'): 
            filo_nesnesi.ds = e
        return {}, {}

    modul = LiderSecimModulu(filo_nesnesi)
    return modul.lideri_belirle(rovlar_data_sozlugu, hedef_konum)


# ==========================================
# LEADER MANAGER MODULE
# ==========================================
class LeaderManager:
    """
    ROV swarm lider yönetim sistemi.
    """
    
    def __init__(self, filo_ref):
        """
        Args:
            filo_ref: Filo referansı
        """
        self.filo_ref = filo_ref
        self.mevcut_lider_id = {}
    
    def guncelle_liderler(self, yeni_lider_ids):
        """
        Lider bilgisini günceller ve rol atamalarını yapar.
        
        Args:
            yeni_lider_ids: {g_id: lider_id} veya {g_id: [lider_id, skor]} formatında olmalı.
        """
        if yeni_lider_ids is None:
            return

        # g_rovs bir sözlük: {g_id: [rov1, rov2, ...]}
        for g_id, rov_listesi in self.filo_ref.g_rovs.items():
            try:
                # 1. VERİ KONTROLÜ (Sözlük uyumlu)
                if isinstance(yeni_lider_ids, dict):
                    if g_id not in yeni_lider_ids:
                        continue
                    raw_val = yeni_lider_ids[g_id]
                else:
                    raw_val = yeni_lider_ids

                # Değer liste/tuple ise ilk elemanı (id) al
                if isinstance(raw_val, (list, tuple, np.ndarray)):
                    yeni_lider_id = raw_val[0]
                else:
                    yeni_lider_id = raw_val

                if not isinstance(yeni_lider_id, int) or yeni_lider_id < 0:
                    continue

                # 2. MEVCUT DURUM KONTROLÜ
                if g_id not in self.mevcut_lider_id:
                    self.mevcut_lider_id[g_id] = -1

                onceki_lider_id = self.mevcut_lider_id[g_id]
                if onceki_lider_id == yeni_lider_id:
                    if self.filo_ref.get(yeni_lider_id, "rol") != 1:
                        self.filo_ref.set(yeni_lider_id, "rol", 1)
                    continue

                print(f"👑 Lider Değişimi | Grup: {g_id} | Yeni Lider: ROV-{yeni_lider_id}")

                # 3. ROLLERİ GÜNCELLE
                self.mevcut_lider_id[g_id] = yeni_lider_id

                for rov in rov_listesi:
                    if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                        continue
                    
                    if rov.id == yeni_lider_id:
                        self.filo_ref.set(rov.id, "rol", 1)
                        from ursina import color
                        rov.color = color.red
                    else:
                        self.filo_ref.set(rov.id, "rol", 0)

                # 4. Patlayan onceki liderin hedefini devret
                if onceki_lider_id not in (None, -1) and onceki_lider_id != yeni_lider_id:
                    eski_lider = self.filo_ref.find_rov_by_id(onceki_lider_id)
                    eski_lider_yok = (eski_lider is None) or (hasattr(eski_lider, 'is_destroyed') and eski_lider.is_destroyed)
                    
                    if eski_lider_yok:
                        # Eski liderin hedefini yeni lidere aktar
                        eski_hedef = self.filo_ref._rov_hedefleri.get(onceki_lider_id)
                        if eski_hedef is not None:
                            self.filo_ref._rov_hedefleri[yeni_lider_id] = eski_hedef
                            self.filo_ref.git(yeni_lider_id, eski_hedef[0], eski_hedef[1], eski_hedef[2], ai=True, sessiz=True)
                            self.filo_ref._rov_hedefleri.pop(onceki_lider_id, None)

                        # Eski liderin rotasını yeni lidere aktar
                        eski_rota = self.filo_ref._git_nokta_listesi.get(onceki_lider_id)
                        if eski_rota:
                            eski_indeks = self.filo_ref._git_mevcut_nokta_indeksi.get(onceki_lider_id, 0)
                            self.filo_ref._git_nokta_listesi[yeni_lider_id] = [list(p) for p in eski_rota]
                            self.filo_ref._git_mevcut_nokta_indeksi[yeni_lider_id] = int(eski_indeks)
                            
                            # Mevcut indexteki waypoint'i hedef yap
                            if 0 <= eski_indeks < len(eski_rota):
                                wp = eski_rota[eski_indeks]
                                hedef_z = eski_hedef[2] if eski_hedef is not None else None
                                self.filo_ref.git(yeni_lider_id, float(wp[0]), float(wp[1]), hedef_z, ai=True, sessiz=True)
                            
                            self.filo_ref._git_nokta_listesi.pop(onceki_lider_id, None)
                            self.filo_ref._git_mevcut_nokta_indeksi.pop(onceki_lider_id, None)

            except Exception as e:
                from .gnc.logs import LogSystem
                LogSystem.log_exception(e)



