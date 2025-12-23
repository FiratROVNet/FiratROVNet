"""
GAT Model Eğitim Scripti - Optimize Edilmiş Versiyon

Bu script, senaryo verilerini kullanarak GAT modelini eğitir.
Veri önbellekleme ile performans optimize edilmiştir.
"""

from FiratROVNet.gat import Train
from FiratROVNet.ortam import veri_uret
from FiratROVNet.config import GATLimitleri
import torch
from torch_geometric.data import Data

# ============================================================
# VERİ ÖNBELLEKLEME (Performans İçin)
# ============================================================
class VeriOnbellek:
    """
    Veri önbellekleme sınıfı - Senaryo modülü ile dinamik veri üretimi.
    Her 500 adımda yeni senaryo oluşturur, her epoch'ta ortamı yeniler.
    """
    def __init__(self, cache_size=100, use_senaryo=False):
        """
        Args:
            cache_size (int): Önbellekte tutulacak veri sayısı
            use_senaryo (bool): Senaryo verilerini kullan (yavaş ama gerçekçi)
        """
        self.cache_size = cache_size
        self.use_senaryo = use_senaryo
        self.cache = []
        self.cache_index = 0
        self.adim_sayaci = 0  # Toplam adım sayacı (500'de bir yeni senaryo için)
        self.epoch_sayaci = 0  # Epoch sayacı
        
        # Senaryo modülü için global instance
        self.senaryo_instance = None
        if self.use_senaryo:
            try:
                from FiratROVNet import senaryo
                self.senaryo_module = senaryo
            except Exception as e:
                print(f"   ⚠️ Senaryo modülü yüklenemedi: {e}")
                self.use_senaryo = False
        
        # Önbelleği doldur
        print(f"📦 Veri önbelleği oluşturuluyor ({cache_size} örnek)...")
        self._fill_cache()
        print(f"✅ Önbellek hazır!")
    
    def _fill_cache(self):
        """Önbelleği doldurur."""
        if self.use_senaryo:
            try:
                # Senaryo verileri için özel üretim
                for i in range(self.cache_size):
                    if i % 10 == 0:
                        print(f"   Veri üretiliyor: {i+1}/{self.cache_size}")
                    # Senaryo verilerini kullan (optimize edilmiş)
                    data = self._veri_uret_senaryo_hizli()
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
    
    def _veri_uret_senaryo_hizli(self):
        """
        Senaryo verilerini hızlı üretir (minimal simülasyon adımları).
        Senaryo açma/kapama maliyetini azaltmak için optimize edilmiştir.
        """
        from FiratROVNet import senaryo
        from torch_geometric.data import Data
        import numpy as np
        import networkx as nx
        
        # Senaryo oluştur (daha az ROV ve engel - hız için)
        n_rovs = np.random.randint(4, 7)  # 4-6 ROV
        n_engels = np.random.randint(8, 15)  # 8-14 engel
        
        try:
            # Senaryo instance'ı yoksa veya yenilenmesi gerekiyorsa oluştur
            if self.senaryo_instance is None or not hasattr(self.senaryo_instance, 'aktif') or not self.senaryo_instance.aktif:
                # Yeni senaryo oluştur
                self.senaryo_instance = self.senaryo_module.uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=200)
            else:
                # Mevcut senaryoyu kullan, sadece pozisyonları güncelle (parametresiz çağrı)
                self.senaryo_module.uret()
            
            # Sadece 1 adım simülasyon (hız için - fizik hesaplamaları minimal)
            self.senaryo_module.guncelle(delta_time=0.016)
            
            # Veri toplama
            rovs = self.senaryo_instance.ortam.rovs
            engeller = self.senaryo_instance.ortam.engeller
            n = len(rovs)
            
            if n == 0:
                self.senaryo_module.temizle()
                self.senaryo_instance = None
                return veri_uret()  # Fallback
            
            x = torch.zeros((n, 7), dtype=torch.float)
            sources, targets = [], []
            danger_map = {}
            
            # GAT limitleri config.py'den alınır (eğitim ve kullanım tutarlılığı için)
            L = {
                'LEADER': GATLimitleri.UZAK,      # 60.0
                'DISCONNECT': GATLimitleri.KOPMA,  # 35.0
                'OBSTACLE': GATLimitleri.ENGEL,    # 20.0
                'COLLISION': GATLimitleri.CARPISMA # 8.0
            }
            
            # Pozisyonları topla (hızlı erişim)
            positions = []
            for rov in rovs:
                if hasattr(rov, 'x'):
                    positions.append([rov.x, getattr(rov, 'y', -2), getattr(rov, 'z', 0)])
                elif hasattr(rov, 'position'):
                    pos = rov.position
                    if hasattr(pos, 'x'):
                        positions.append([pos.x, pos.y, pos.z])
                    else:
                        positions.append([0, -2, 0])
                else:
                    positions.append([0, -2, 0])
            
            # GAT girdilerini oluştur (optimize edilmiş)
            positions_np = np.array([p[:2] for p in positions])  # Sadece X, Z
            
            for i in range(n):
                code = 0
                pos_i = positions_np[i]
                
                # Liderden uzak mı?
                if i != 0:
                    if np.linalg.norm(pos_i - positions_np[0]) > L['LEADER']:
                        code = 5
                
                # Diğer ROV'lardan uzak mı? (vektörel hesaplama)
                dists = np.linalg.norm(positions_np - pos_i, axis=1)
                dists[i] = np.inf  # Kendisini hariç tut
                min_dist = np.min(dists)
                
                if min_dist > L['DISCONNECT']:
                    code = 3
                else:
                    # Edge'leri ekle
                    for j in range(n):
                        if i != j and dists[j] < L['DISCONNECT']:
                            sources.append(i)
                            targets.append(j)
                
                # Engel kontrolü (sadece yakın engeller)
                min_engel_dist = 999.0
                for engel in engeller[:15]:  # İlk 15 engel (hız için)
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
                
                if min_engel_dist < L['OBSTACLE']:
                    code = 1
                
                # Çarpışma kontrolü (vektörel)
                collision_mask = (dists < L['COLLISION']) & (dists > 0)
                if np.any(collision_mask):
                    code = 2
                
                # GAT özellik vektörü
                x[i][0] = code / 5.0
                
                # Batarya
                if hasattr(rovs[i], 'battery'):
                    bat = rovs[i].battery
                    x[i][1] = float(bat) if bat <= 1.0 else bat / 100.0
                else:
                    x[i][1] = 0.8
                
                x[i][2] = 0.9  # SNR
                x[i][3] = abs(float(positions[i][1])) / 100.0 if len(positions[i]) > 1 else 0.5
                
                # Hız
                if hasattr(rovs[i], 'velocity'):
                    vel = rovs[i].velocity
                    x[i][4] = float(getattr(vel, 'x', 0.0))
                    x[i][5] = float(getattr(vel, 'z', 0.0))
                else:
                    x[i][4] = 0.0
                    x[i][5] = 0.0
                
                x[i][6] = float(getattr(rovs[i], 'role', 1.0 if i == 0 else 0.0))
                
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
                    priority = {2: 0, 1: 1, 3: 2, 5: 3, 0: 4}
                    sorted_dangers = sorted(danger_map.items(), key=lambda k: priority.get(k[1], 10))
                    for d_node, d_code in sorted_dangers:
                        if nx.has_path(G, i, d_node):
                            y[i] = d_code
                            break
            
            # Senaryoyu temizleme (her 500 adımda veya epoch başında)
            # Senaryo instance'ı korunur, sadece pozisyonlar güncellenir
            
            return Data(x=x, edge_index=edge_index, y=y)
            
        except Exception as e:
            # Hata durumunda fallback
            try:
                if hasattr(self, 'senaryo_module'):
                    self.senaryo_module.temizle()
                    self.senaryo_instance = None
            except:
                pass
            return veri_uret()
    
    def __call__(self):
        """
        Önbellekten veri döndürür (round-robin).
        Her 500 adımda yeni senaryo oluşturur, her epoch'ta ortamı yeniler.
        """
        # Epoch başında (cache_index == 0) veya her 500 adımda yeni senaryo oluştur
        if self.use_senaryo and (self.cache_index == 0 or self.adim_sayaci % 500 == 0):
            if self.adim_sayaci % 500 == 0 and self.adim_sayaci > 0:
                print(f"   🔄 Yeni senaryo oluşturuluyor (Adım: {self.adim_sayaci})...")
            
            # Senaryo instance'ını yenile
            try:
                if self.senaryo_instance is not None:
                    self.senaryo_module.temizle()
                self.senaryo_instance = None
                
                # Yeni senaryo oluştur
                import numpy as np
                n_rovs = np.random.randint(4, 7)  # 4-6 ROV
                n_engels = np.random.randint(8, 15)  # 8-14 engel
                self.senaryo_instance = self.senaryo_module.uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=200)
            except Exception as e:
                print(f"   ⚠️ Senaryo yenilenemedi: {e}")
        
        # Epoch başında önbelleği yenile
        if self.cache_index == 0:
            self.epoch_sayaci += 1
            if self.epoch_sayaci > 1:  # İlk epoch'ta önbellek zaten dolu
                if self.use_senaryo:
                    print(f"   🔄 Epoch {self.epoch_sayaci}: Senaryo önbelleği yenileniyor...")
                    # Önbelleği yeniden doldur (yeni senaryolarla)
                    self.cache = []
                    for i in range(min(10, self.cache_size)):  # Her epoch'ta 10 yeni veri
                        data = self._veri_uret_senaryo_hizli()
                        self.cache.append(data)
                else:
                    # Sentetik veri için de önbelleği yenile
                    self.cache = []
                    for i in range(self.cache_size):
                        self.cache.append(veri_uret())
        
        # Veriyi önbellekten al
        data = self.cache[self.cache_index]
        self.cache_index = (self.cache_index + 1) % len(self.cache)
        self.adim_sayaci += 1
        
        return data


# ============================================================
# EĞİTİM
# ============================================================
if __name__ == "__main__":
    print("🚀 GAT Model Eğitimi Başlıyor...")
    print("=" * 60)
    
    # Senaryo verileriyle eğitim (yeni sistem)
    print("\n📦 Senaryo Modülü ile Eğitim")
    print("   - Her 500 adımda yeni senaryo oluşturulacak")
    print("   - Her epoch'ta ortam yenilenecek")
    
    veri_kaynagi_senaryo = VeriOnbellek(cache_size=50, use_senaryo=True)
    
    print("\n🎯 Eğitim Başlıyor...")
    Train(veri_kaynagi=veri_kaynagi_senaryo, epochs=10000, lr=0.002)
    
    # Senaryoyu temizle
    if hasattr(veri_kaynagi_senaryo, 'senaryo_instance') and veri_kaynagi_senaryo.senaryo_instance:
        try:
            veri_kaynagi_senaryo.senaryo_module.temizle()
        except:
            pass
    
    print("\n✅ Eğitim tamamlandı!")
    print(f"📁 Model kaydedildi: rov_modeli_multi.pth")
