import torch
import numpy as np
import networkx as nx
from torch_geometric.data import Data

def veri_uret(n_rovs=None):
    """
    FıratROVNet için optimize edilmiş sentetik eğitim ve test verisi üretir.
    GAT kodlarını daha iyi öğrenmesi için çeşitli senaryolar ve dengeli veri dağılımı sağlar.
    
    Parametre: n_rovs (int) -> ROV sayısı (Boş bırakılırsa rastgele 4-15 arası)
    """
    if n_rovs is None:
        n_rovs = np.random.randint(4, 15)
    
    # Limitler (GAT kodları için)
    LIMITLER = {
        'CARPISMA': 8.0,   # Kod 2: Çarpışma riski
        'ENGEL': 20.0,     # Kod 1: Engel yakınlığı
        'KOPMA': 35.0,     # Kod 3: Bağlantı kopması
        'UZAK': 60.0       # Kod 5: Liderden uzaklık
    }
    
    # Senaryo tipi seçimi (daha çeşitli öğrenme için)
    senaryo_tipi = np.random.choice([
        'formasyon',      # ROV'lar birbirine yakın (iyi bağlantı)
        'dagınık',        # ROV'lar dağınık (kopma riski)
        'tehlikeli',      # Çok engel var (engel yakınlığı)
        'uzak_lider',     # Takipçiler liderden uzak
        'carpisma_risk',  # Çarpışma riski yüksek
        'karma'           # Karışık senaryo
    ], p=[0.2, 0.15, 0.2, 0.15, 0.15, 0.15])
    
    # Pozisyon üretimi (senaryo tipine göre)
    havuz_genisligi = 200.0
    pos = np.zeros((n_rovs, 2))
    
    if senaryo_tipi == 'formasyon':
        # Lider merkezde, takipçiler yakın
        lider_pos = np.array([havuz_genisligi/2, havuz_genisligi/2])
        pos[0] = lider_pos
        for i in range(1, n_rovs):
            angle = 2 * np.pi * i / (n_rovs - 1)
            radius = np.random.uniform(15, 30)  # Yakın formasyon
            pos[i] = lider_pos + radius * np.array([np.cos(angle), np.sin(angle)])
            
    elif senaryo_tipi == 'dagınık':
        # ROV'lar geniş alana dağılmış
        for i in range(n_rovs):
            pos[i] = np.random.uniform(0, havuz_genisligi, 2)
            
    elif senaryo_tipi == 'tehlikeli':
        # ROV'lar engel bölgelerine yakın
        engel_merkezleri = [
            np.array([havuz_genisligi*0.3, havuz_genisligi*0.3]),
            np.array([havuz_genisligi*0.7, havuz_genisligi*0.7]),
            np.array([havuz_genisligi*0.5, havuz_genisligi*0.2])
        ]
        for i in range(n_rovs):
            engel_merkez = np.random.choice(len(engel_merkezleri))
            center = engel_merkezleri[engel_merkez]
            # Engel merkezine yakın ama rastgele dağılım
            pos[i] = center + np.random.uniform(-40, 40, 2)
            pos[i] = np.clip(pos[i], 10, havuz_genisligi - 10)
            
    elif senaryo_tipi == 'uzak_lider':
        # Lider merkezde, takipçiler uzakta
        pos[0] = np.array([havuz_genisligi/2, havuz_genisligi/2])
        for i in range(1, n_rovs):
            angle = 2 * np.pi * np.random.random()
            radius = np.random.uniform(65, 90)  # Uzak mesafe
            pos[i] = pos[0] + radius * np.array([np.cos(angle), np.sin(angle)])
            pos[i] = np.clip(pos[i], 10, havuz_genisligi - 10)
            
    elif senaryo_tipi == 'carpisma_risk':
        # ROV'lar birbirine çok yakın
        merkez = np.array([havuz_genisligi/2, havuz_genisligi/2])
        for i in range(n_rovs):
            angle = 2 * np.pi * np.random.random()
            radius = np.random.uniform(3, 12)  # Çok yakın
            pos[i] = merkez + radius * np.array([np.cos(angle), np.sin(angle)])
            
    else:  # karma
        # Karışık dağılım
        for i in range(n_rovs):
            pos[i] = np.random.uniform(0, havuz_genisligi, 2)
    
    # Çoklu engel merkezleri (daha gerçekçi)
    n_engeller = np.random.randint(2, 5)
    engel_merkezleri = []
    for _ in range(n_engeller):
        engel_merkezleri.append(np.random.uniform(20, havuz_genisligi-20, 2))
    engel_merkezleri = np.array(engel_merkezleri)
    
    # GAT girdileri ve tehlikeler
    x = torch.zeros((n_rovs, 7), dtype=torch.float)
    danger_map = {}
    
    # Mesafe matrisi (optimizasyon için)
    dist_matrix = np.zeros((n_rovs, n_rovs))
    for i in range(n_rovs):
        for j in range(n_rovs):
            dist_matrix[i, j] = np.linalg.norm(pos[i] - pos[j])
    
    # Her ROV için GAT kodu hesaplama
    for i in range(n_rovs):
        code = 0
        
        # 1. Çarpışma kontrolü (en yüksek öncelik)
        min_rov_dist = np.min([dist_matrix[i, j] for j in range(n_rovs) if j != i])
        if min_rov_dist < LIMITLER['CARPISMA']:
            code = 2
        
        # 2. Engel yakınlığı kontrolü
        if code == 0:  # Çarpışma yoksa engel kontrolü yap
            min_engel_dist = float('inf')
            for engel_pos in engel_merkezleri:
                dist = np.linalg.norm(pos[i] - engel_pos)
                min_engel_dist = min(min_engel_dist, dist)
            
            if min_engel_dist < LIMITLER['ENGEL']:
                code = 1
        
        # 3. Bağlantı kopması kontrolü
        if code == 0:  # Daha düşük öncelikli tehlikeler
            if min_rov_dist > LIMITLER['KOPMA']:
                code = 3
        
        # 4. Liderden uzaklık kontrolü (sadece takipçiler için)
        if code == 0 and i != 0:
            lider_dist = dist_matrix[i, 0]
            if lider_dist > LIMITLER['UZAK']:
                code = 5
        
        # GAT kodu özelliği
        x[i][0] = code / 5.0
        if code > 0:
            danger_map[i] = code
    
    # Gerçekçi sensör değerleri (birbiriyle ilişkili)
    for i in range(n_rovs):
        # Batarya (0.1 - 1.0, düşük batarya daha az olası)
        battery_base = np.random.beta(2, 1)  # Beta dağılımı (yüksek değerlere yönelik)
        x[i][1] = np.clip(battery_base, 0.1, 1.0)
        
        # SNR (batarya ile ilişkili: düşük batarya = düşük SNR)
        snr_base = 0.5 + battery_base * 0.5
        x[i][2] = np.clip(snr_base + np.random.uniform(-0.1, 0.1), 0.3, 1.0)
        
        # Derinlik (0.0 - 1.0, normalize edilmiş)
        depth = np.random.uniform(0, 100)  # 0-100 metre
        x[i][3] = depth / 100.0
        
        # Hız (batarya ve duruma göre)
        # Düşük batarya veya tehlike varsa hız düşük
        if x[i][1] < 0.3 or code > 0:
            speed_factor = 0.3
        else:
            speed_factor = 1.0
        
        # Hız vektörü (normalize edilmiş)
        speed_magnitude = np.random.uniform(0, 1) * speed_factor
        angle = np.random.uniform(0, 2 * np.pi)
        x[i][4] = speed_magnitude * np.cos(angle)  # Vx
        x[i][5] = speed_magnitude * np.sin(angle)  # Vz
        
        # Rol (0 = takipçi, 1 = lider)
        x[i][6] = 1.0 if i == 0 else 0.0
    
    # Graf bağlantıları (iletişim menzili bazlı)
    sources, targets = [], []
    iletişim_menzili = LIMITLER['KOPMA']  # GATLimitleri.KOPMA (35.0 birim)
    
    for i in range(n_rovs):
        for j in range(n_rovs):
            if i != j and dist_matrix[i, j] < iletişim_menzili:
                # SNR bazlı bağlantı olasılığı (düşük SNR = bağlantı yok)
                snr_i = x[i][2].item()
                snr_j = x[j][2].item()
                connection_prob = (snr_i + snr_j) / 2.0
                
                if np.random.random() < connection_prob:
                    sources.append(i)
                    targets.append(j)
    
    edge_index = torch.tensor([sources, targets], dtype=torch.long) if sources else torch.zeros((2, 0), dtype=torch.long)
    
    # Hedef etiketler (Y) - Graph yayılımı ile
    y = torch.zeros(n_rovs, dtype=torch.long)
    G = nx.Graph()
    G.add_nodes_from(range(n_rovs))
    if len(sources) > 0:
        G.add_edges_from(zip(sources, targets))
    
    # Öncelik sırası: Çarpışma > Engel > Kopma > Uzak > OK
    priority = {2: 0, 1: 1, 3: 2, 5: 3, 0: 4}
    
    for i in range(n_rovs):
        if i in danger_map:
            # Doğrudan tehlike
            y[i] = danger_map[i]
        elif i in G.nodes() and len(danger_map) > 0:
            # Graph üzerinden yayılan tehlike
            sorted_dangers = sorted(danger_map.items(), key=lambda k: priority.get(k[1], 10))
            for d_node, d_code in sorted_dangers:
                try:
                    if nx.has_path(G, i, d_node):
                        y[i] = d_code
                        break
                except:
                    pass
        else:
            # Güvenli durum
            y[i] = 0
    
    return Data(x=x, edge_index=edge_index, y=y)
