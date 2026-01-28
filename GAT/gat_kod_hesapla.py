"""
GAT Kod Hesaplayıcı
Config'den limitleri çekerek GAT kodlarını hesaplar.
"""

import sys
import os

# Proje kök dizinini path'e ekle
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import numpy as np
import torch
import networkx as nx
from FiratROVNet.config import GATLimitleri


def hesapla_gat_kodlari(graf_verisi, senaryo_verisi):
    """
    GAT kodlarını hesaplar ve graf yayılımı ile etiketleri oluşturur.
    
    Args:
        graf_verisi (dict): gat_graf.olustur_graf() fonksiyonundan dönen veri
        senaryo_verisi (dict): gat_veri_uret() fonksiyonundan dönen veri
    
    Returns:
        dict: {
            'gat_kodlari': dict,  # {rov_id: gat_kodu}
            'y': torch.Tensor,  # Hedef etiketler (n_rovs,)
            'x': torch.Tensor  # Güncellenmiş özellik matrisi (GAT kodları ile)
        }
    """
    senaryo = senaryo_verisi['senaryo']
    x = graf_verisi['x']
    dist_matrix = graf_verisi['dist_matrix']
    n_rovs = graf_verisi['n_rovs']
    
    danger_map = {}  # {rov_id: gat_kodu}
    
    # GAT kodlarını hesapla (gerçek durumlara göre)
    for i in range(n_rovs):
        code = 0
        
        # 1. Çarpışma kontrolü (en yüksek öncelik)
        min_rov_dist = np.min([dist_matrix[i, j] for j in range(n_rovs) if j != i])
        if min_rov_dist < GATLimitleri.CARPISMA:
            code = 2
            danger_map[i] = code
        
        # 2. Engel yakınlığı kontrolü (sonar verisi)
        if code == 0:
            sonar = senaryo.get(i, 'sonar')
            if sonar is not None and sonar > 0 and sonar < GATLimitleri.ENGEL:
                code = 1
                danger_map[i] = code
        
        # 3. Bağlantı kopması kontrolü
        if code == 0:
            if min_rov_dist > GATLimitleri.KOPMA:
                code = 3
                danger_map[i] = code
        
        # 4. Liderden uzaklık kontrolü (sadece takipçiler için)
        if code == 0:
            # Lider ID'yi bul
            lider_id = None
            for j in range(n_rovs):
                rol = senaryo.get(j, 'rol')
                if rol == 1:
                    lider_id = j
                    break
            
            if lider_id is None:
                lider_id = 0  # Fallback: ilk ROV lider
            
            if i != lider_id:
                lider_dist = dist_matrix[i, lider_id]
                if lider_dist > GATLimitleri.UZAK:
                    code = 4  # GAT kodu 4 = UZAK
                    danger_map[i] = code
        
        # GAT kodu özelliği (normalize edilmiş)
        x[i][0] = float(code / 5.0)
    
    return {
        'gat_kodlari': danger_map,
        'x': x
    }


def yayilim_etiketleri_olustur(graf_verisi, gat_kodlari_dict, edge_index, graph_nx):
    """
    Graf yayılımı ile etiketleri oluşturur.
    
    Args:
        graf_verisi (dict): gat_graf.olustur_graf() fonksiyonundan dönen veri
        gat_kodlari_dict (dict): {rov_id: gat_kodu}
        edge_index (torch.Tensor): Graf bağlantıları
        graph_nx (nx.Graph): NetworkX grafı
    
    Returns:
        torch.Tensor: y (n_rovs,) - Hedef etiketler
    """
    n_rovs = graf_verisi['n_rovs']
    y = torch.zeros(n_rovs, dtype=torch.long)
    
    # Öncelik sırası: Çarpışma > Engel > Kopma > Uzak > OK
    priority = {2: 0, 1: 1, 3: 2, 4: 3, 0: 4}
    
    for i in range(n_rovs):
        if i in gat_kodlari_dict:
            # Doğrudan tehlike
            y[i] = gat_kodlari_dict[i]
        elif i in graph_nx.nodes() and len(gat_kodlari_dict) > 0:
            # Graph üzerinden yayılan tehlike
            sorted_dangers = sorted(gat_kodlari_dict.items(), key=lambda k: priority.get(k[1], 10))
            for d_node, d_code in sorted_dangers:
                try:
                    if nx.has_path(graph_nx, i, d_node):
                        y[i] = d_code
                        break
                except:
                    pass
        else:
            # Güvenli durum
            y[i] = 0
    
    return y


def tam_veri_olustur(senaryo_verisi):
    """
    Senaryo verilerinden tam GAT verisini oluşturur (graf + kodlar + etiketler).
    
    Args:
        senaryo_verisi (dict): gat_veri_uret() fonksiyonundan dönen veri
    
    Returns:
        torch_geometric.data.Data: GAT modeli için hazırlanmış veri
    """
    from GAT.gat_graf import olustur_graf, olustur_edge_index, olustur_graph_nx
    
    # Graf yapısını oluştur
    graf_verisi = olustur_graf(senaryo_verisi)
    
    # GAT kodlarını hesapla
    kod_verisi = hesapla_gat_kodlari(graf_verisi, senaryo_verisi)
    
    # Edge index oluştur
    edge_index = olustur_edge_index(
        graf_verisi['dist_matrix'],
        GATLimitleri.KOPMA,
        kod_verisi['x']
    )
    
    # NetworkX grafı oluştur
    graph_nx = olustur_graph_nx(edge_index, graf_verisi['n_rovs'])
    
    # Yayılım etiketleri oluştur
    y = yayilim_etiketleri_olustur(
        graf_verisi,
        kod_verisi['gat_kodlari'],
        edge_index,
        graph_nx
    )
    
    # PyTorch Geometric Data objesi oluştur
    from torch_geometric.data import Data
    return Data(x=kod_verisi['x'], edge_index=edge_index, y=y)
