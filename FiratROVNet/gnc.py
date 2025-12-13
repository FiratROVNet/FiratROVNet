import numpy as np
from ursina import Vec3, time
from .config import cfg
import math

# ==========================================
# 1. YÖNETİCİ SINIF (FİLO KOMUTANI)
# ==========================================
class GNCKomutan:
    def __init__(self):
        self.sistemler = [] 

    def ekle(self, gnc_objesi):
        self.sistemler.append(gnc_objesi)

    def rehber_dagit(self, modem_rehberi):
        if self.sistemler:
            for sistem in self.sistemler:
                if isinstance(sistem, LiderGNC):
                    sistem.rehber_guncelle(modem_rehberi)

    def guncelle_hepsi(self, tahminler):
        for i, gnc in enumerate(self.sistemler):
            if i < len(tahminler):
                gnc.guncelle(tahminler[i])

    # --- GÜNCELLENEN GİT FONKSİYONU ---
    def git(self, rov_id, x, z, y=None, ai=True):
        """
        KONSOL KOMUTU: Hedefe git.
        Parametreler: (ID, X, Z, Y=Derinlik, ai=True/False)
        """
        if 0 <= rov_id < len(self.sistemler):
            # Manuel modu kapat, otopilotu aç
            self.sistemler[rov_id].manuel_kontrol = False
            
            # AI Durumunu Ayarla
            self.sistemler[rov_id].ai_aktif = ai
            
            # Hedef Ata
            hedef_y = y if y is not None else self.sistemler[rov_id].rov.y
            
            ai_durum = "AÇIK" if ai else "KAPALI (Kör Mod)"
            print(f"🔵 [KOMUTAN] ROV-{rov_id} Rota: {x}, {z}, {hedef_y} | AI: {ai_durum}")
            
            self.sistemler[rov_id].hedef_atama(x, hedef_y, z)
        else:
            print(f"❌ [HATA] Geçersiz ROV ID: {rov_id}")

# ==========================================
# 2. TEMEL GNC SINIFI
# ==========================================
class TemelGNC:
    def __init__(self, rov_entity, modem):
        self.rov = rov_entity
        self.modem = modem
        self.hedef = None 
        self.hiz_limiti = 100.0 
        self.manuel_kontrol = False
        
        # YENİ: Bireysel AI Anahtarı
        self.ai_aktif = True 

    def hedef_atama(self, x, y, z):
        self.hedef = Vec3(x, y, z)

    def rehber_guncelle(self, rehber):
        if self.modem: self.modem.rehber_guncelle(rehber)

    def vektor_to_motor(self, vektor, guc_carpani=1.0):
        if vektor.length() == 0: return

        guc = self.hiz_limiti * guc_carpani

        if vektor.x > 0.1: self.rov.move("sag", abs(vektor.x) * guc)
        elif vektor.x < -0.1: self.rov.move("sol", abs(vektor.x) * guc)

        if vektor.y > 0.1: self.rov.move("cik", abs(vektor.y) * guc)
        elif vektor.y < -0.1: self.rov.move("bat", abs(vektor.y) * guc)

        if vektor.z > 0.1: self.rov.move("ileri", abs(vektor.z) * guc)
        elif vektor.z < -0.1: self.rov.move("geri", abs(vektor.z) * guc)

# ==========================================
# 3. LİDER VE TAKİPÇİ (AI KONTROLLÜ)
# ==========================================
class LiderGNC(TemelGNC):
    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return 
        if self.hedef is None: return
        
        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, gelen uyarıyı görmezden gel (0 kabul et)
        if not self.ai_aktif:
            gat_kodu = 0
        
        mevcut = self.rov.position
        fark = self.hedef - mevcut
        if fark.length() < 1.0: return

        if self.hedef.y < 0: self.hedef.y = 0
        yon = fark.normalized()

        if gat_kodu == 1: yon += Vec3(1, 0, 0) 
        elif gat_kodu == 2: yon = Vec3(0, 0, 0)

        self.vektor_to_motor(yon)

class TakipciGNC(TemelGNC):
    def __init__(self, rov_entity, modem, lider_modem_ref=None):
        super().__init__(rov_entity, modem)
        self.lider_ref = lider_modem_ref

    def guncelle(self, gat_kodu):
        if self.manuel_kontrol: return
        if self.hedef is None: return

        # --- AI KONTROLÜ ---
        # Eğer AI kapalıysa, tehlike yokmuş gibi (0) davran
        if not self.ai_aktif:
            gat_kodu = 0

        fark = self.hedef - self.rov.position
        if fark.length() < 1.5: return
        
        hedef_vektoru = fark.normalized()
        kacinma_vektoru = Vec3(0,0,0)

        # GAT Tepkileri
        if gat_kodu == 1: 
            kacinma_vektoru = Vec3(0, 1.0, 0) + (hedef_vektoru * -0.5)
        elif gat_kodu == 2: 
            kacinma_vektoru = -hedef_vektoru * 1.5
        elif gat_kodu == 3: 
            kacinma_vektoru = Vec3(0, 0.2, 0) 
        elif gat_kodu == 5: 
            pass

        # Vektör Birleştirme
        if gat_kodu != 0 and gat_kodu != 5: 
            nihai_vektor = kacinma_vektoru + (hedef_vektoru * 0.1)
        else:
            nihai_vektor = hedef_vektoru

        guc = 1.0
        if gat_kodu == 5: guc = 1.5 
        if gat_kodu == 1: guc = 0.5 
        
        self.vektor_to_motor(nihai_vektor, guc_carpani=guc)
