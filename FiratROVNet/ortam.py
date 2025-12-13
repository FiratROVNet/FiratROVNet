import torch
import numpy as np
import networkx as nx
from torch_geometric.data import Data

def veri_uret(n_rovs=None):
    """
    FıratROVNet için sentetik eğitim ve test verisi üretir.
    Parametre: n_rovs (int) -> ROV sayısı (Boş bırakılırsa rastgele 4-15 arası)
    """
    if n_rovs is None:
        n_rovs = np.random.randint(4, 15)
        
    pos = np.random.rand(n_rovs, 2) * 100
    danger_center = np.array([50, 50])
    
    x = torch.zeros((n_rovs, 7), dtype=torch.float)
    danger_map = {} 
    
    # Limitler
    LIMITLER = {'CARPISMA': 8.0, 'ENGEL': 20.0, 'KOPMA': 30.0, 'UZAK': 60.0}
    
    # --- GİRDİLERİ OLUŞTUR ---
    for i in range(n_rovs):
        code = 0
        # Sensör Kontrolleri
        if i != 0 and np.linalg.norm(pos[i] - pos[0]) > LIMITLER['UZAK']: code = 5
        
        dists = [np.linalg.norm(pos[i] - pos[j]) for j in range(n_rovs) if i != j]
        if dists and min(dists) > LIMITLER['KOPMA']: code = 3
            
        if np.linalg.norm(pos[i] - danger_center) < LIMITLER['ENGEL']: code = 1
            
        x[i][0] = code / 5.0
        if code > 0: danger_map[i] = code

    # Çarpışma (Baskın Durum)
    for i in range(n_rovs):
        for j in range(i + 1, n_rovs):
            if np.linalg.norm(pos[i] - pos[j]) < LIMITLER['CARPISMA']:
                x[i][0] = 2.0 / 5.0
                x[j][0] = 2.0 / 5.0
                danger_map[i] = 2
                danger_map[j] = 2

    # Diğer Sensörler (Pil, SNR, Derinlik, Hızlar, Rol)
    for i in range(n_rovs):
        x[i][1] = np.random.uniform(0.1, 1.0) 
        x[i][2] = np.random.uniform(0.5, 1.0) 
        x[i][3] = np.random.uniform(0.0, 1.0) 
        x[i][4] = np.random.uniform(-1, 1)    
        x[i][5] = np.random.uniform(-1, 1)    
        x[i][6] = 1.0 if i == 0 else 0.0      

    # --- GRAF BAĞLANTILARI ---
    sources, targets = [], []
    for i in range(n_rovs):
        for j in range(n_rovs):
            if i != j and np.linalg.norm(pos[i] - pos[j]) < 35:
                sources.append(i); targets.append(j)
    edge_index = torch.tensor([sources, targets], dtype=torch.long)
    
    # --- HEDEF ETİKETLER (Y) ---
    y = torch.zeros(n_rovs, dtype=torch.long)
    G = nx.Graph()
    G.add_nodes_from(range(n_rovs))
    if len(sources) > 0: G.add_edges_from(zip(sources, targets))
    
    for i in range(n_rovs):
        if i in danger_map:
            y[i] = danger_map[i]
        elif i in G:
            priority = {2:0, 1:1, 3:2, 5:3, 0:4}
            sorted_dangers = sorted(danger_map.items(), key=lambda k: priority.get(k[1], 10))
            for d_node, d_code in sorted_dangers:
                if nx.has_path(G, i, d_node):
                    y[i] = d_code
                    break
                    
    return Data(x=x, edge_index=edge_index, y=y)
