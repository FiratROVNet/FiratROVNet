"""
GAT Test Modülü
Eğitilmiş GAT modelini test eder.
"""

import sys
import os

# Proje kök dizinini path'e ekle
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import numpy as np
from GAT.gat_train import GAT_Modeli
from FiratROVNet.gnc import Filo

MODEL_DOSYA_ADI = "rov_modeli_multi.pth"


class FiratAnalizci:
    """
    Fırat Üniversitesi için geliştirilmiş GAT Tabanlı ROV Analiz Sınıfı.
    """
    def __init__(self, model_yolu=MODEL_DOSYA_ADI):
        self.device = torch.device('cpu')
        self.model = GAT_Modeli().to(self.device)
        
        print(f"🔹 Analizci Başlatılıyor...")

        if os.path.exists(model_yolu):
            try:
                self.model.load_state_dict(torch.load(model_yolu, map_location=self.device))
                print(f"✅ Model Yüklendi: {model_yolu}")
            except Exception as e:
                print(f"❌ Model Hata: {e}")
        else:
            print(f"⚠️ Uyarı: '{model_yolu}' bulunamadı! Rastgele çalışacak.")
        
        self.model.eval()

    def analiz_et(self, veri):
        """
        GAT verisini analiz eder ve tahminleri döndürür.
        
        Args:
            veri: torch_geometric.data.Data objesi
        
        Returns:
            tuple: (tahminler, edge_idx, alpha)
        """
        with torch.no_grad():
            out, edge_idx, alpha = self.model(veri.x, veri.edge_index, return_attention=True)
            tahminler = out.argmax(dim=1).numpy()
        return tahminler, edge_idx, alpha


def test(n_samples=100):
    """
    GAT modelini test eder.
    
    Args:
        n_samples (int): Test örnek sayısı
    
    Returns:
        dict: Test sonuçları
    """
    print(f"🧪 GAT Test Başlıyor... ({n_samples} örnek)")
    
    # Analizci oluştur
    analizci = FiratAnalizci()
    
    # Filo instance'ı oluştur
    filo = Filo()
    
    # İstatistikler
    toplam_rov = 0
    toplam_edge = 0
    dogru_tahmin = 0
    toplam_tahmin = 0
    gat_kodlari_dagilimi = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    tahmin_dagilimi = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
    
    print()
    print("=" * 80)
    print("TEST BAŞLIYOR")
    print("=" * 80)
    print()
    
    for i in range(n_samples):
        # Senaryo verisi üret
        senaryo_verisi = filo.gat_veri_uret()
        if senaryo_verisi is None:
            continue
        
        # GAT verisini oluştur
        try:
            from GAT.gat_kod_hesapla import tam_veri_olustur
            data = tam_veri_olustur(senaryo_verisi)
        except Exception as e:
            print(f"   ⚠️ Örnek {i+1}: GAT verisi oluşturulamadı: {e}")
            continue
        
        # İstatistikler
        toplam_rov += data.x.shape[0]
        toplam_edge += data.edge_index.shape[1]
        
        # Tahmin yap
        tahminler, _, _ = analizci.analiz_et(data)
        
        # Doğruluk hesapla
        gercek = data.y.numpy()
        dogru_tahmin += np.sum(tahminler == gercek)
        toplam_tahmin += len(gercek)
        
        # Dağılım hesapla
        for code in gercek:
            if int(code) in gat_kodlari_dagilimi:
                gat_kodlari_dagilimi[int(code)] += 1
        
        for code in tahminler:
            if int(code) in tahmin_dagilimi:
                tahmin_dagilimi[int(code)] += 1
        
        if (i + 1) % 10 == 0:
            accuracy = dogru_tahmin / toplam_tahmin if toplam_tahmin > 0 else 0.0
            print(f"   🔹 Örnek {i+1}/{n_samples} | Doğruluk: {accuracy:.2%} | ROV: {toplam_rov} | Edge: {toplam_edge}")
    
    print()
    print("=" * 80)
    print("✅ TEST TAMAMLANDI")
    print("=" * 80)
    
    accuracy = dogru_tahmin / toplam_tahmin if toplam_tahmin > 0 else 0.0
    
    print(f"   Toplam örnek: {n_samples}")
    print(f"   Toplam ROV: {toplam_rov}")
    print(f"   Ortalama ROV/örnek: {toplam_rov / n_samples:.2f}")
    print(f"   Toplam Edge: {toplam_edge}")
    print(f"   Ortalama Edge/örnek: {toplam_edge / n_samples:.2f}")
    print(f"   Doğruluk: {accuracy:.2%}")
    print(f"   Doğru tahmin: {dogru_tahmin}/{toplam_tahmin}")
    print()
    print(f"   📈 Gerçek GAT Kodları Dağılımı:")
    for code, count in gat_kodlari_dagilimi.items():
        yuzde = (count / toplam_rov * 100) if toplam_rov > 0 else 0.0
        print(f"      Kod {code}: {count} ({yuzde:.1f}%)")
    print()
    print(f"   🎯 Tahmin GAT Kodları Dağılımı:")
    for code, count in tahmin_dagilimi.items():
        yuzde = (count / toplam_rov * 100) if toplam_rov > 0 else 0.0
        print(f"      Kod {code}: {count} ({yuzde:.1f}%)")
    print("=" * 80)
    print()
    
    return {
        'accuracy': accuracy,
        'toplam_rov': toplam_rov,
        'toplam_edge': toplam_edge,
        'gat_kodlari_dagilimi': gat_kodlari_dagilimi,
        'tahmin_dagilimi': tahmin_dagilimi
    }


if __name__ == "__main__":
    import os
    # Test başlat
    sonuclar = test(n_samples=100)
