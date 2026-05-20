import math
import random
import numpy as np

class LiderSecimModulu:
    def __init__(self, filo_ref):
        self.filo_ref = filo_ref

    def _grup_hedefini_coz(self, hedef_konum, g_id):
        if isinstance(hedef_konum, dict):
            return hedef_konum.get(g_id)
        return hedef_konum

    def _gecerli_mevcut_lider_id(self, g_id, rov_listesi):
        leader_manager = getattr(self.filo_ref, "leader_manager", None)
        if leader_manager is not None:
            mevcut_liderler = getattr(leader_manager, "mevcut_lider_id", {})
            lider_id = mevcut_liderler.get(g_id)
            if isinstance(lider_id, int) and lider_id >= 0:
                for rov in rov_listesi:
                    if rov.get("id") == lider_id:
                        return lider_id

        lider_bilgi = self.filo_ref.find_leader_info(sessiz=True, g_id=g_id)
        lider_id = lider_bilgi[0] if lider_bilgi else None
        if isinstance(lider_id, int) and lider_id >= 0:
            return lider_id
        return None

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
            grup_hedef = self._grup_hedefini_coz(hedef_konum, g_id)

            # Grup boşsa atla
            if not rov_listesi:
                secilen_rov_id[g_id] = -1
                lider_skorlari[g_id] = 0
                continue

            # --- DURUM 0: Gecerli bir mevcut lider varsa yeniden secim yapma ---
            # Lider hayattaysa ve grup listesinde hala varsa, hedef olsa bile korunur.
            mevcut_lider_id = self._gecerli_mevcut_lider_id(g_id, rov_listesi)
            if mevcut_lider_id is not None:
                secilen_rov_id[g_id] = mevcut_lider_id
                lider_skorlari[g_id] = 1.0
                continue

            # --- DURUM A: HEDEF YOKSA (Mevcut lideri koru veya random seç) ---
            if grup_hedef is None:
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
                    p3 = self.deger_duzenle(self.a_star_simulasyonu(rov['konum'], grup_hedef))
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
    hedef_konum:
    - None
    - tek hedef tuple/list
    - veya {g_id: hedef} biçiminde grup bazlı sözlük
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

    def _gnc_mod_ata(self, rov, mod):
        if hasattr(rov, "gnc") and rov.gnc is not None:
            try:
                rov.gnc.mod = int(mod)
            except Exception:
                pass
    
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
                    for rov in rov_listesi:
                        if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                            continue
                        if rov.id == yeni_lider_id:
                            self._gnc_mod_ata(rov, 0)
                    continue

                print(f"👑 Lider Değişimi | Grup: {g_id} | Yeni Lider: ROV-{yeni_lider_id}")

                # 3. ROLLERİ GÜNCELLE
                self.mevcut_lider_id[g_id] = yeni_lider_id

                for rov in rov_listesi:
                    if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                        continue
                    
                    if rov.id == yeni_lider_id:
                        self.filo_ref.set(rov.id, "rol", 1)
                        # Lider: serbest mod (mod=0) — minimap hedefleri ve git_path alabilsin
                        self._gnc_mod_ata(rov, 0)
                    else:
                        self.filo_ref.set(rov.id, "rol", 0)

                # 4. Patlayan onceki liderin hedefini devret
                if onceki_lider_id not in (None, -1) and onceki_lider_id != yeni_lider_id:
                    eski_lider = self.filo_ref.find_rov_by_id(onceki_lider_id)
                    eski_lider_yok = (eski_lider is None) or (hasattr(eski_lider, 'is_destroyed') and eski_lider.is_destroyed)
                    
                    if eski_lider_yok:
                        miras = getattr(self.filo_ref, '_olum_mirasi', {}).get(onceki_lider_id, {})
                        eski_hedef = miras.get('hedef')

                        eski_rota = miras.get('rota')
                        if eski_rota:
                            eski_indeks = miras.get('indeks', 0)
                            
                            if 0 <= eski_indeks < len(eski_rota):
                                yeni_lider_gps = self.filo_ref.get(yeni_lider_id, "gps")
                                eski_lider_konum = miras.get('konum')
                                hedef_z = miras.get('derinlik')
                                if hedef_z is None and eski_hedef is not None and len(eski_hedef) >= 3:
                                    hedef_z = eski_hedef[2]
                                
                                if yeni_lider_gps:
                                    baslangic_2d = (float(yeni_lider_gps[0]), float(yeni_lider_gps[1]))
                                    if eski_lider_konum is not None and len(eski_lider_konum) >= 2:
                                        hedef_2d = (float(eski_lider_konum[0]), float(eski_lider_konum[1]))
                                    else:
                                        wp = eski_rota[eski_indeks]
                                        hedef_2d = (float(wp[0]), float(wp[1]))
                                    
                                    # Yeni liderden patlama konumuna A* baglanti ciz; sonra eski rotanin kalanini ekle.
                                    baglanti_yolu = []
                                    try:
                                        path_planla = getattr(self.filo_ref, "_a_star_path_planla", None)
                                        if path_planla is None:
                                            path_planla = getattr(getattr(self.filo_ref, "helper", None), "_a_star_path_planla", None)
                                        if callable(path_planla):
                                            baglanti_yolu = path_planla(baslangic_2d, hedef_2d, duzlem_z=yeni_lider_gps[2])
                                    except Exception:
                                        pass
                                        
                                    if not baglanti_yolu:
                                        baglanti_yolu = [baslangic_2d, hedef_2d]
                                        
                                    kalan_rota = [list(p) for p in eski_rota[eski_indeks:]]
                                    yeni_rota = [list(p) for p in baglanti_yolu] + kalan_rota
                                    
                                    # Yeni rotayi ata (git() fonksiyonu bunu _git_nokta_listesi'ne koyacak)
                                    self.filo_ref.git(yeni_lider_id, yeni_rota, z=hedef_z, ai=True, sessiz=True)
                                    
                                    # Minimap path guncellemesi (opsiyonel ama gorsel icin iyi olur)
                                    ortam = getattr(self.filo_ref, "ortam_ref", None)
                                    if ortam and hasattr(ortam, "minimap") and ortam.minimap:
                                        try:
                                            ortam.minimap.update_path(yeni_rota)
                                        except Exception:
                                            pass
                            
                            self.filo_ref._git_nokta_listesi.pop(onceki_lider_id, None)
                            self.filo_ref._git_mevcut_nokta_indeksi.pop(onceki_lider_id, None)
                            if hasattr(self.filo_ref, '_git_hedef_derinligi'):
                                self.filo_ref._git_hedef_derinligi.pop(onceki_lider_id, None)
                        elif eski_hedef is not None:
                            self.filo_ref._rov_hedefleri[yeni_lider_id] = eski_hedef
                            self.filo_ref.git(yeni_lider_id, eski_hedef[0], eski_hedef[1], eski_hedef[2], ai=True, sessiz=True)

            except Exception as e:
                from .gnc.logs import LogSystem
                LogSystem.log_exception(e)
