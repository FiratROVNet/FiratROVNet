from .simulasyon import Ortam
from .gat import FiratAnalizci
from .iletisim import AkustikModem
from .gnc import GNCKomutan, LiderGNC, TakipciGNC
from .config import cfg
from ursina import *
import torch
import numpy as np
import os
import sys

class ROV_Sistemi:
    def __init__(self, n_rovs=4, n_engels=15, model_yolu="rov_modeli_multi.pth"):
        print("🔵 Fırat-GNC Sistemi Hazırlanıyor...")
        
        # 1. Ortam ve Simülasyon
        self.app = Ortam()
        self.app.sim_olustur(n_rovs, n_engels)
        
        # 2. Yapay Zeka
        try:
            self.beyin = FiratAnalizci(model_yolu)
        except:
            print("⚠️ Model yüklenemedi, AI devre dışı.")
            self.beyin = None

        # 3. GNC (Komuta Kontrol)
        self.komutan = GNCKomutan()
        
        # 4. Sistemi Kur (Donanımları tak)
        self._donanim_kurulumu()
        
        # 5. Konsol ve Döngü Bağlantıları
        self.app.konsola_ekle("git", self.komutan.git) # 'git' komutunu buraya bağlıyoruz
        self.app.konsola_ekle("sistem", self)
        
        # Update ve Input fonksiyonlarını Ursina'ya bağla
        self.app.set_update_function(self._ana_dongu)
        self.app.app.input = self._klavye_kontrolu

    def _donanim_kurulumu(self):
        """ROV'lara modem takar, rolleri dağıtır ve GNC'yi başlatır."""
        tum_modemler = {}
        lider_modem = AkustikModem(rov_id=0, gurultu_orani=0.05)
        tum_modemler[0] = lider_modem
        
        for i, rov in enumerate(self.app.rovs):
            if i == 0: # Lider
                rov.set("rol", 1)
                rov.modem = lider_modem
                gnc = LiderGNC(rov, lider_modem)
                self.komutan.ekle(gnc)
                self.komutan.git(0, 40, 60, 0) # Başlangıç hedefi
            else: # Takipçi
                rov.set("rol", 0)
                modem = AkustikModem(rov_id=i, gurultu_orani=0.1)
                rov.modem = modem
                tum_modemler[i] = modem
                gnc = TakipciGNC(rov, modem, lider_modem_ref=lider_modem)
                self.komutan.ekle(gnc)
                self.komutan.git(i, 30 + (i*5), 50, -10) # Formasyon
        
        self.komutan.rehber_dagit(tum_modemler)

    def baslat(self):
        print("✅ Sistem Başlatılıyor... (Konsol Arka Planda)")
        try:
            self.app.run(interaktif=True)
        except KeyboardInterrupt:
            pass
        finally:
            os.system('stty sane')
            os._exit(0)

    def _simden_veriye(self):
        """Fiziksel dünyayı Matematiksel matrise çevirir (GAT Girdisi)"""
        rovs = self.app.rovs
        engeller = self.app.engeller
        n = len(rovs)
        x = torch.zeros((n, 7), dtype=torch.float)
        positions = [r.position for r in rovs]
        sources, targets = [], []

        L = {'LEADER': 60.0, 'DISCONNECT': 35.0, 'OBSTACLE': 20.0, 'COLLISION': 8.0}

        for i in range(n):
            code = 0
            if i != 0 and distance(positions[i], positions[0]) > L['LEADER']: code = 5
            dists = [distance(positions[i], positions[j]) for j in range(n) if i != j]
            if dists and min(dists) > L['DISCONNECT']: code = 3
            
            min_engel = 999
            for engel in engeller:
                d = distance(positions[i], engel.position) - 6 
                if d < min_engel: min_engel = d
            if min_engel < L['OBSTACLE']: code = 1
            
            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['COLLISION']:
                    code = 2; break
            
            x[i][0] = code / 5.0
            x[i][1] = rovs[i].battery / 100.0
            x[i][2] = 0.9
            x[i][3] = abs(rovs[i].y) / 100.0
            x[i][4] = rovs[i].velocity.x
            x[i][5] = rovs[i].velocity.z
            x[i][6] = rovs[i].role

            for j in range(n):
                if i != j and distance(positions[i], positions[j]) < L['DISCONNECT']:
                    sources.append(i); targets.append(j)

        edge_index = torch.tensor([sources, targets], dtype=torch.long)
        class MiniData:
            def __init__(self, x, edge_index): self.x, self.edge_index = x, edge_index
        return MiniData(x, edge_index)

    def _ana_dongu(self):
        """Saniyede 60 kez çalışan ana mantık"""
        try:
            # 1. Veri Topla
            veri = self._simden_veriye()
            
            # 2. AI Tahmini
            ai_aktif = getattr(cfg, 'ai_aktif', True)
            if ai_aktif and self.beyin:
                try: tahminler, _, _ = self.beyin.analiz_et(veri)
                except: tahminler = np.zeros(len(self.app.rovs), dtype=int)
            else:
                tahminler = np.zeros(len(self.app.rovs), dtype=int)

            # 3. Görselleştirme
            kod_renkleri = {0:color.orange, 1:color.red, 2:color.black, 3:color.yellow, 5:color.magenta}
            durum_txts = ["OK", "ENGEL", "CARPISMA", "KOPUK", "-", "UZAK"]
            
            for i, gat_kodu in enumerate(tahminler):
                if self.app.rovs[i].role == 1:
                    self.app.rovs[i].color = color.red
                else:
                    self.app.rovs[i].color = kod_renkleri.get(gat_kodu, color.white)
                
                ek = "" if ai_aktif else "\n[AI OFF]"
                self.app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}{ek}"
                
            # 4. Komutanı Bilgilendir (Aksiyon)
            self.komutan.guncelle_hepsi(tahminler)
            
        except Exception as e:
            pass # Hata olursa simülasyonu durdurma

    def _klavye_kontrolu(self, key):
        if key == 'escape' or key == 'q':
            os.system('stty sane') 
            application.quit()
            os._exit(0)
