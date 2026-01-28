"""
GAT Graf Yapısı Oluşturucu
Senaryo verilerinden graf yapısını oluşturur.
"""

import sys
import os

# Proje kök dizinini path'e ekle
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import numpy as np
from torch_geometric.data import Data
import networkx as nx


def olustur_graf(senaryo_verisi):
    """
    Senaryo verilerinden graf yapısını oluşturur.
    
    Args:
        senaryo_verisi (dict): gat_veri_uret() fonksiyonundan dönen veri
            {
                'senaryo': Senaryo instance,
                'filo': Filo instance,
                'ortam': Ortam instance,
                'n_rovs': int,
                'n_adalar': int,
                'n_engels': int
            }
    
    Returns:
        dict: {
            'x': torch.Tensor,  # Özellik matrisi (n_rovs, 7)
            'positions': list,  # ROV pozisyonları
            'dist_matrix': np.ndarray,  # Mesafe matrisi
            'n_rovs': int
        }
    """
    senaryo = senaryo_verisi['senaryo']
    n_rovs = senaryo_verisi['n_rovs']
    
    # Özellik matrisi (7 özellik: GAT_kodu, batarya, SNR, derinlik, vx, vz, rol)
    x = torch.zeros((n_rovs, 7), dtype=torch.float)
    
    # Pozisyon matrisi (mesafe hesaplamaları için)
    positions = []
    
    # Her ROV için veri topla
    for i in range(n_rovs):
        # GPS koordinatları
        gps = senaryo.get(i, 'gps')
        if gps is None:
            gps = np.array([0.0, 0.0, -5.0])
        elif isinstance(gps, (list, tuple)):
            gps = np.array(gps)
        positions.append(gps[:2])  # Sadece x, y (2D düzlem)
        
        # Batarya
        battery = senaryo.get(i, 'batarya')
        if battery is None:
            battery = 1.0
        x[i][1] = float(np.clip(battery, 0.0, 1.0))
        
        # SNR (batarya ile ilişkili, gerçekçi simülasyon)
        snr = 0.5 + float(battery) * 0.5 + np.random.uniform(-0.1, 0.1)
        x[i][2] = float(np.clip(snr, 0.3, 1.0))
        
        # Derinlik (z koordinatı, normalize edilmiş)
        depth = abs(float(gps[2])) if len(gps) > 2 else 5.0
        x[i][3] = float(np.clip(depth / 100.0, 0.0, 1.0))
        
        # Hız vektörü
        velocity = senaryo.get(i, 'hiz')
        if velocity is None:
            velocity = np.array([0.0, 0.0, 0.0])
        elif isinstance(velocity, (list, tuple)):
            velocity = np.array(velocity)
        # Hızı normalize et (0-1 arası)
        speed_magnitude = np.linalg.norm(velocity[:2])  # Sadece x, y bileşenleri
        speed_magnitude = np.clip(speed_magnitude / 10.0, 0.0, 1.0)  # 10 m/s maksimum
        if speed_magnitude > 0.01:
            angle = np.arctan2(velocity[1], velocity[0])
            x[i][4] = float(speed_magnitude * np.cos(angle))  # Vx
            x[i][5] = float(speed_magnitude * np.sin(angle))  # Vz
        else:
            x[i][4] = 0.0
            x[i][5] = 0.0
        
        # Rol (lider = 1.0, takipçi = 0.0)
        role = senaryo.get(i, 'rol')
        if role is None:
            role = 0
        x[i][6] = 1.0 if role == 1 else 0.0
    
    # Mesafe matrisi
    dist_matrix = np.zeros((n_rovs, n_rovs))
    for i in range(n_rovs):
        for j in range(n_rovs):
            if i != j:
                dist_matrix[i, j] = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
    
    return {
        'x': x,
        'positions': positions,
        'dist_matrix': dist_matrix,
        'n_rovs': n_rovs
    }


def olustur_edge_index(dist_matrix, iletişim_menzili, x):
    """
    Graf bağlantılarını (edge_index) oluşturur.
    
    Args:
        dist_matrix (np.ndarray): Mesafe matrisi
        iletişim_menzili (float): İletişim menzili (metre)
        x (torch.Tensor): Özellik matrisi (SNR bilgisi için)
    
    Returns:
        torch.Tensor: edge_index (2, num_edges)
    """
    sources, targets = [], []
    n_rovs = dist_matrix.shape[0]
    
    for i in range(n_rovs):
        for j in range(n_rovs):
            if i != j and dist_matrix[i, j] < iletişim_menzili:
                # SNR bazlı bağlantı olasılığı
                snr_i = x[i][2].item()
                snr_j = x[j][2].item()
                connection_prob = (snr_i + snr_j) / 2.0
                
                if np.random.random() < connection_prob:
                    sources.append(i)
                    targets.append(j)
    
    edge_index = torch.tensor([sources, targets], dtype=torch.long) if sources else torch.zeros((2, 0), dtype=torch.long)
    return edge_index


def olustur_graph_nx(edge_index, n_rovs):
    """
    NetworkX grafı oluşturur (yayılım için).
    
    Args:
        edge_index (torch.Tensor): Graf bağlantıları
        n_rovs (int): ROV sayısı
    
    Returns:
        nx.Graph: NetworkX grafı
    """
    G = nx.Graph()
    G.add_nodes_from(range(n_rovs))
    
    if edge_index.shape[1] > 0:
        sources = edge_index[0].numpy()
        targets = edge_index[1].numpy()
        G.add_edges_from(zip(sources, targets))
    
    return G
