"""
GAT Model Eğitimi Scripti - Optimize Edilmiş Versiyon
======================================================

Bu script, senaryo verilerini kullanarak GAT modelini eğitir.
25 epoch'ta bir yeni ortam oluşturur, diğer epoch'larda pozisyonları random olarak günceller.
Tüm ayarlar config.py'den alınır.
"""

import sys
import os

# Proje kök dizinini path'e ekle
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from FiratROVNet.gat import Train
from FiratROVNet.ortam import veri_uret
from FiratROVNet.config import GATLimitleri
import torch
from torch_geometric.data import Data
import numpy as np
import networkx as nx
import math
import random
import time


# ============================================================
# VERİ ÖNBELLEKLEME (Performans İçin)
# ============================================================
class VeriOnbellek:
    """
    Veri önbellekleme sınıfı - Senaryo modülü ile dinamik veri üretimi.
    25 epoch'ta bir yeni ortam oluşturur, diğer epoch'larda pozisyonları random günceller.
    Tüm GAT limitleri config.py'den alınır.
    """
    def __init__(self, cache_size=50, use_senaryo=True):
        """
        Args:
            cache_size (int): Önbellekte tutulacak veri sayısı
            use_senaryo (bool): Senaryo verilerini kullan (yavaş ama gerçekçi)
        """
        self.cache_size = cache_size
        self.use_senaryo = use_senaryo
        self.cache = []
        self.cache_index = 0
        self.epoch_sayaci = 0  # Epoch sayacı
        
        # Senaryo modülü için global instance referansı
        self.senaryo_instance = None  # Senaryo instance referansı
        self.senaryo_module = None  # Senaryo modül referansı
        self._ilk_ortam_olusturuldu = False  # İlk ortam oluşturuldu mu?
        self._yeni_ortam_olusturuldu = False  # Bu epoch'ta yeni ortam oluşturuldu mu?
        
        # Config'den GAT limitlerini al (HER ZAMAN CONFIG'DEN)
        self.gat_limits = {
            'CARPISMA': GATLimitleri.CARPISMA,
            'ENGEL': GATLimitleri.ENGEL,
            'KOPMA': GATLimitleri.KOPMA,
            'UZAK': GATLimitleri.UZAK
        }
        
        # Havuz genişliği
        self.havuz_genisligi = 200.0
        
        if self.use_senaryo:
            try:
                from FiratROVNet import senaryo
                self.senaryo_module = senaryo
                # Global instance'ı al (eğer varsa)
                if hasattr(senaryo, '_senaryo_instance') and senaryo._senaryo_instance is not None:
                    self.senaryo_instance = senaryo._senaryo_instance
            except Exception as e:
                print(f"   ⚠️ Senaryo modülü yüklenemedi: {e}")
                self.use_senaryo = False
        
        # Önbelleği doldur
        print(f"📦 Veri önbelleği oluşturuluyor ({cache_size} örnek)...")
        self._fill_cache()
        print(f"✅ Önbellek hazır!")
    
    def _fill_cache(self):
        """Önbelleği doldurur (optimize edilmiş versiyon)."""
        if self.use_senaryo:
            try:
                # İlk veri için ortam oluştur, sonraki veriler için mevcut ortamı kullan
                for i in range(self.cache_size):
                    if i % 10 == 0:
                        print(f"   Veri üretiliyor: {i+1}/{self.cache_size}")
                    # İlk veri için ortam oluştur, sonraki için sadece pozisyonları güncelle
                    data = self._veri_uret_senaryo(cache_doldurma_modu=True, ilk_veri=(i == 0))
                    self.cache.append(data)
            except Exception as e:
                print(f"   ⚠️ Senaryo yüklenemedi, ortam.veri_uret kullanılıyor: {e}")
                self.use_senaryo = False
                for i in range(self.cache_size):
                    self.cache.append(veri_uret())
        else:
            # Hızlı sentetik veri
            for i in range(self.cache_size):
                self.cache.append(veri_uret())
    
    def _veri_uret_senaryo(self, cache_doldurma_modu=False, ilk_veri=False):
        """
        Senaryo verilerini üretir (optimize edilmiş versiyon).
        25 epoch'ta bir yeni ortam oluşturur, diğer epoch'larda senaryo.py'nin 
        optimize edilmiş pozisyon güncelleme mekanizmasını kullanır.
        Tüm GAT limitleri config.py'den alınır.
        
        Args:
            cache_doldurma_modu (bool): Cache doldurma modunda mı? (epoch kontrolü yapma)
            ilk_veri (bool): İlk veri mi? (ortam oluştur)
        """
        if not self.use_senaryo or self.senaryo_module is None:
            return veri_uret()
        
        try:
            # Senaryo instance kontrolü ve güncelleme
            if self.senaryo_instance is None:
                # Global instance'ı kontrol et
                if hasattr(self.senaryo_module, '_senaryo_instance') and self.senaryo_module._senaryo_instance is not None:
                    self.senaryo_instance = self.senaryo_module._senaryo_instance
                else:
                    # Instance yoksa yeni ortam oluştur
                    n_rovs = np.random.randint(4, 10)  # 4-9 ROV
                    n_engels = np.random.randint(10, 25)  # 10-24 engel
                    self.senaryo_module.uret(
                        n_rovs=n_rovs,
                        n_engels=n_engels,
                        havuz_genisligi=self.havuz_genisligi,
                        verbose=False  # Log mesajlarını gizle
                    )
                    self.senaryo_instance = self.senaryo_module._senaryo_instance
                    self._ilk_ortam_olusturuldu = True
            
            # 25 epoch'ta bir yeni ortam oluştur (cache doldurma modunda değilse)
            if cache_doldurma_modu:
                # Cache doldurma modu: İlk veri için ortam oluştur, sonraki için mevcut ortamı kullan
                if ilk_veri and not self._ilk_ortam_olusturuldu:
                    # İlk ortam zaten yukarıda oluşturuldu, sadece pozisyonları güncelle
                    self.senaryo_instance._nesneleri_yeniden_dagit()  # Sadece pozisyon güncelleme
                    self._ilk_ortam_olusturuldu = True
                else:
                    # Mevcut ortamı kullan, sadece pozisyonları güncelle (ÇOK HIZLI!)
                    self.senaryo_instance._nesneleri_yeniden_dagit()  # Sadece pozisyon güncelleme
            else:
                # Normal mod: 25 epoch'ta bir yeni ortam oluştur
                if self.epoch_sayaci % 25 == 0:
                    # 25. epoch'ta yeni ortam oluştur (sadece flag False ise)
                    if not self._yeni_ortam_olusturuldu:
                        # Yeni ortam oluştur (sayıları değiştir) - Sadece bir kez!
                        n_rovs = np.random.randint(4, 10)  # 4-9 ROV
                        n_engels = np.random.randint(10, 25)  # 10-24 engel
                        self.senaryo_module.uret(
                            n_rovs=n_rovs,
                            n_engels=n_engels,
                            havuz_genisligi=self.havuz_genisligi,
                            verbose=False  # Log mesajlarını gizle
                        )
                        self.senaryo_instance = self.senaryo_module._senaryo_instance
                        self._yeni_ortam_olusturuldu = True  # Flag'i set et
                    else:
                        # Yeni ortam zaten oluşturuldu, sadece pozisyonları güncelle
                        self.senaryo_instance._nesneleri_yeniden_dagit()  # Sadece pozisyon güncelleme
                else:
                    # Mevcut ortamı kullan, sadece pozisyonları güncelle (ÇOK HIZLI!)
                    self.senaryo_instance._nesneleri_yeniden_dagit()  # Sadece pozisyon güncelleme
            
            # Veri toplama
            ortam = self.senaryo_instance.ortam
            rovs = ortam.rovs if hasattr(ortam, 'rovs') else []
            engeller = ortam.engeller if hasattr(ortam, 'engeller') else []
            n = len(rovs)
            
            if n == 0:
                return veri_uret()  # Fallback
            
            x = torch.zeros((n, 7), dtype=torch.float)
            sources, targets = [], []
            danger_map = {}
            
            # Pozisyonları topla
            positions = []
            lider_id = 0
            for i, rov in enumerate(rovs):
                if hasattr(rov, 'position'):
                    pos = rov.position
                    if hasattr(pos, 'x'):
                        positions.append([pos.x, pos.y, pos.z])
                    else:
                        positions.append([0, -2, 0])
                elif hasattr(rov, 'x'):
                    positions.append([rov.x, getattr(rov, 'y', -2), getattr(rov, 'z', 0)])
                else:
                    positions.append([0, -2, 0])
                
                # Lider ID'yi bul
                if hasattr(rov, 'role') and rov.role == 1:
                    lider_id = i
                elif hasattr(rov, 'get'):
                    rol = rov.get('rol')
                    if rol == 1:
                        lider_id = i
            
            # GAT girdilerini oluştur (CONFIG'DEN LİMİTLER KULLANILIR)
            positions_np = np.array([p[:2] for p in positions])  # Sadece X, Z
            
            for i in range(n):
                code = 0
                pos_i = positions_np[i]
                
                # Liderden uzak mı? (Config'den - GATLimitleri.UZAK)
                if i != lider_id:
                    lider_pos = positions_np[lider_id]
                    if np.linalg.norm(pos_i - lider_pos) > self.gat_limits['UZAK']:
                        code = 4  # GAT kodu 4 = UZAK
                
                # Diğer ROV'lardan uzak mı? (vektörel hesaplama - Config'den)
                dists = np.linalg.norm(positions_np - pos_i, axis=1)
                dists[i] = np.inf  # Kendisini hariç tut
                min_dist = np.min(dists)
                
                if min_dist > self.gat_limits['KOPMA']:
                    code = 3  # GAT kodu 3 = KOPUK
                else:
                    # Edge'leri ekle (Config'den - GATLimitleri.KOPMA)
                    for j in range(n):
                        if i != j and dists[j] < self.gat_limits['KOPMA']:
                            sources.append(i)
                            targets.append(j)
                
                # Engel kontrolü (config'den - GATLimitleri.ENGEL)
                min_engel_dist = 999.0
                for engel in engeller:
                    try:
                        if hasattr(engel, 'position'):
                            engel_pos = engel.position
                            if hasattr(engel_pos, 'x'):
                                engel_x, engel_z = engel_pos.x, engel_pos.z
                            else:
                                continue
                            dist = np.linalg.norm(pos_i - np.array([engel_x, engel_z]))
                            if dist < min_engel_dist:
                                min_engel_dist = dist
                    except:
                        continue
                
                if min_engel_dist < self.gat_limits['ENGEL']:
                    code = 1  # GAT kodu 1 = ENGEL
                
                # Çarpışma kontrolü (vektörel, config'den - GATLimitleri.CARPISMA)
                collision_mask = (dists < self.gat_limits['CARPISMA']) & (dists > 0)
                if np.any(collision_mask):
                    code = 2  # GAT kodu 2 = CARPISMA
                
                # GAT özellik vektörü
                x[i][0] = code / 5.0
                
                # Batarya
                if hasattr(rovs[i], 'battery'):
                    bat = rovs[i].battery
                    x[i][1] = float(bat) if bat <= 1.0 else bat / 100.0
                else:
                    x[i][1] = np.random.uniform(0.5, 1.0)  # Random batarya
                
                x[i][2] = np.random.uniform(0.7, 1.0)  # SNR
                x[i][3] = abs(float(positions[i][1])) / 100.0 if len(positions[i]) > 1 else 0.5
                
                # Hız
                if hasattr(rovs[i], 'velocity'):
                    vel = rovs[i].velocity
                    x[i][4] = float(getattr(vel, 'x', 0.0))
                    x[i][5] = float(getattr(vel, 'z', 0.0))
                else:
                    x[i][4] = np.random.uniform(-1, 1)
                    x[i][5] = np.random.uniform(-1, 1)
                
                # Rol (lider mi?)
                if i == lider_id:
                    x[i][6] = 1.0
                else:
                    x[i][6] = 0.0
                
                if code > 0:
                    danger_map[i] = code
            
            edge_index = torch.tensor([sources, targets], dtype=torch.long) if sources else torch.zeros((2, 0), dtype=torch.long)
            
            # Hedef etiketler
            y = torch.zeros(n, dtype=torch.long)
            G = nx.Graph()
            G.add_nodes_from(range(n))
            if len(sources) > 0:
                G.add_edges_from(zip(sources, targets))
            
            for i in range(n):
                if i in danger_map:
                    y[i] = danger_map[i]
                elif i in G and len(danger_map) > 0:
                    priority = {2: 0, 1: 1, 3: 2, 4: 3, 0: 4}
                    sorted_dangers = sorted(danger_map.items(), key=lambda k: priority.get(k[1], 10))
                    for d_node, d_code in sorted_dangers:
                        if nx.has_path(G, i, d_node):
                            y[i] = d_code
                            break
            
            return Data(x=x, edge_index=edge_index, y=y)
            
        except Exception as e:
            # Hata durumunda fallback
            print(f"   ⚠️ Senaryo veri üretim hatası: {e}")
            try:
                # Senaryo instance'ı yeniden oluştur
                if self.senaryo_instance is None or not hasattr(self.senaryo_instance, 'ortam'):
                    # Global instance'ı kontrol et
                    if hasattr(self.senaryo_module, '_senaryo_instance') and self.senaryo_module._senaryo_instance is not None:
                        self.senaryo_instance = self.senaryo_module._senaryo_instance
                    else:
                        # Yeni ortam oluştur
                        n_rovs = np.random.randint(4, 10)
                        n_engels = np.random.randint(10, 25)
                        self.senaryo_module.uret(
                            n_rovs=n_rovs,
                            n_engels=n_engels,
                            havuz_genisligi=self.havuz_genisligi,
                            verbose=False
                        )
                        self.senaryo_instance = self.senaryo_module._senaryo_instance
                        self._ilk_ortam_olusturuldu = True
            except Exception as e2:
                print(f"   ⚠️ Senaryo instance yeniden oluşturulamadı: {e2}")
            return veri_uret()
    
    def __call__(self):
        """
        Önbellekten veri döndürür (round-robin).
        25 epoch'ta bir yeni ortam oluşturur, diğer epoch'larda pozisyonları random günceller.
        """
        # Epoch başında önbelleği yenile
        if self.cache_index == 0:
            self.epoch_sayaci += 1
            self._yeni_ortam_olusturuldu = False  # Her epoch başında flag'i sıfırla
            
            if self.epoch_sayaci > 1:  # İlk epoch'ta önbellek zaten dolu
                if self.use_senaryo:
                    # 25 epoch'ta bir yeni ortam oluştur, diğer epoch'larda mevcut ortamı kullan
                    if self.epoch_sayaci % 25 == 0:
                        print(f"   🔄 Epoch {self.epoch_sayaci}: Yeni ortam oluşturuluyor...")
                        # Yeni ortam oluşturulacak, önbelleği yeniden doldur
                        self.cache = []
                        for i in range(min(10, self.cache_size)):  # Her 25 epoch'ta 10 yeni veri
                            if i == 0:
                                # İlk veri: Yeni ortam oluştur
                                self._yeni_ortam_olusturuldu = True  # Flag'i set et
                                data = self._veri_uret_senaryo(cache_doldurma_modu=False)  # Normal mod (25 epoch kontrolü yapılacak)
                            else:
                                # Sonraki veriler: Mevcut ortamı kullan, sadece pozisyonları güncelle
                                data = self._veri_uret_senaryo(cache_doldurma_modu=True, ilk_veri=False)  # Cache modu (sadece pozisyon güncelleme)
                            self.cache.append(data)
                    else:
                        # Mevcut ortamı kullan, sadece pozisyonları güncelle (ÇOK HIZLI!)
                        # Önbelleği yeniden doldur (mevcut ortamla)
                        self.cache = []
                        for i in range(min(10, self.cache_size)):  # Her epoch'ta 10 yeni veri
                            data = self._veri_uret_senaryo(cache_doldurma_modu=True, ilk_veri=False)  # Cache modu (sadece pozisyon güncelleme)
                            self.cache.append(data)
                else:
                    # Sentetik veri için de önbelleği yenile
                    self.cache = []
                    for i in range(self.cache_size):
                        self.cache.append(veri_uret())
        
        # Veriyi önbellekten al
        data = self.cache[self.cache_index]
        self.cache_index = (self.cache_index + 1) % len(self.cache)
        
        return data


# ============================================================
# EĞİTİM
# ============================================================
def main():
    print("=" * 80)
    print("🚀 GAT MODEL EĞİTİMİ BAŞLATILIYOR")
    print("=" * 80)
    print()
    print("📊 Eğitim Parametreleri:")
    print("   - Epochs: 500")
    print("   - Senaryo Modu: Aktif (gerçek simülasyon verileri)")
    print("   - Learning Rate: 0.001")
    print("   - Hidden Channels: 16")
    print("   - Attention Heads: 4")
    print("   - Dropout: 0.1")
    print("   - Weight Decay: 1e-4")
    print("   - Device: CUDA (eğer mevcut) veya CPU")
    print()
    print("📝 Log Detayları:")
    print("   - Her epoch: Loss ve Accuracy")
    print("   - Her 10 epoch: Detaylı bilgi")
    print("   - Her 30 epoch: GAT dağılımı ve gradient norm (tam detay)")
    print("   - Her 100 epoch: Veri üretim bilgisi (ROV, edge, Ada sayıları)")
    print()
    print("🔄 Ortam Oluşturma:")
    print("   - Her 25 epoch'ta bir yeni ortam oluşturulur")
    print("   - Diğer epoch'larda sadece pozisyonlar yeniden dağıtılır")
    print()
    print("⚙️ GAT Limitleri (Config'den):")
    print(f"   - CARPISMA: {GATLimitleri.CARPISMA}m")
    print(f"   - ENGEL: {GATLimitleri.ENGEL}m")
    print(f"   - KOPMA: {GATLimitleri.KOPMA}m")
    print(f"   - UZAK: {GATLimitleri.UZAK}m")
    print()
    print("=" * 80)
    print()
    
    baslangic_zamani = time.time()
    
    try:
        # Veri önbelleği oluştur (gat_train.py mantığı)
        print("\n📦 Senaryo Modülü ile Eğitim")
        print("   - 25 epoch'ta bir yeni ortam oluşturulacak")
        print("   - Diğer epoch'larda pozisyonlar random güncellenecek")
        print("   - Tüm GAT limitleri config.py'den alınacak")
        print()
        
        veri_kaynagi = VeriOnbellek(cache_size=50, use_senaryo=True)
        
        print("\n🎯 Eğitim Başlıyor...")
        
        # GAT modelini eğit
        model, best_loss = Train(
            veri_kaynagi=veri_kaynagi,  # Veri önbelleği kullan
            epochs=10000,
            lr=0.001,
            hidden_channels=16,
            num_heads=4,
            dropout=0.1,
            weight_decay=1e-4,
            use_senaryo=False  # Veri önbelleği zaten senaryo kullanıyor
        )
        
        toplam_sure = time.time() - baslangic_zamani
        
        # Senaryoyu temizle
        if hasattr(veri_kaynagi, 'senaryo_instance') and veri_kaynagi.senaryo_instance:
            try:
                if hasattr(veri_kaynagi, 'senaryo_module') and veri_kaynagi.senaryo_module:
                    if hasattr(veri_kaynagi.senaryo_module, 'temizle'):
                        veri_kaynagi.senaryo_module.temizle()
            except:
                pass
        
        print()
        print("=" * 80)
        print("✅ EĞİTİM BAŞARIYLA TAMAMLANDI!")
        print("=" * 80)
        print(f"   ⏱️  Toplam süre: {toplam_sure / 60:.2f} dakika ({toplam_sure:.2f} saniye)")
        print(f"   📉 En düşük loss: {best_loss:.4f}")
        print(f"   💾 Model dosyası: rov_modeli_multi.pth")
        print(f"   📊 Ortalama epoch süresi: {toplam_sure / 500:.3f} saniye")
        print()
        print("🎯 Model kullanıma hazır!")
        print("=" * 80)
        print()
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 80)
        print("⚠️ EĞİTİM KULLANICI TARAFINDAN İPTAL EDİLDİ")
        print("=" * 80)
        print("   Mevcut model kaydedilmiş olabilir: rov_modeli_multi.pth")
        print("   Eğitime devam etmek için scripti tekrar çalıştırabilirsiniz.")
        print("=" * 80)
        print()
    except Exception as e:
        print("\n\n" + "=" * 80)
        print("❌ EĞİTİM SIRASINDA HATA OLUŞTU")
        print("=" * 80)
        print(f"   Hata: {e}")
        print()
        print("📋 Detaylı Hata Bilgisi:")
        print("-" * 80)
        import traceback
        traceback.print_exc()
        print("=" * 80)
        print()


if __name__ == "__main__":
    main()

