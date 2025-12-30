import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch_geometric.nn import GATConv
from torch_geometric.data import Data
import os
import numpy as np
import networkx as nx
from .ortam import veri_uret
from .config import GATLimitleri

MODEL_DOSYA_ADI = "rov_modeli_multi.pth"


# ============================================================
# GAT VERİ ÜRETİCİ SINIFI
# ============================================================
class GATVeriUretici:
    """
    GAT eğitimi için gerçek simülasyon verileri üreten sınıf.
    Senaryo modülünü kullanarak gerçekçi ROV sensör verileri ve GAT kodları oluşturur.
    """
    
    def __init__(self):
        """GAT veri üreticisini başlatır."""
        self._senaryo_instance = None
        self._ortam_olusturma_sayaci = 0  # Her 50 örnekte bir yeni ortam için sayaç
    
    def _get_senaryo_instance(self):
        """Senaryo instance'ını alır veya oluşturur."""
        if self._senaryo_instance is None:
            try:
                from . import senaryo
                self._senaryo_instance = senaryo._get_instance()
            except ImportError:
                self._senaryo_instance = None
        return self._senaryo_instance
    
    def veri_uret(self, n_rovs=None, n_engels=None, havuz_genisligi=None):
        """
        GAT eğitimi için gerçek simülasyon verileri üretir.
        Senaryo ortamını kullanarak gerçekçi veri toplar.
        
        Args:
            n_rovs (int, optional): ROV sayısı (None ise rastgele 4-15)
            n_engels (int, optional): Engel sayısı (None ise rastgele 10-25)
            havuz_genisligi (float, optional): Havuz genişliği (None ise 200)
        
        Returns:
            torch_geometric.data.Data: GAT modeli için hazırlanmış veri
            
        Özellikler:
            - x: (n_rovs, 7) özellik matrisi
                [0]: GAT_kodu (normalize edilmiş)
                [1]: Batarya (0-1)
                [2]: SNR (0.3-1.0)
                [3]: Derinlik (normalize edilmiş)
                [4]: Hız X bileşeni (normalize edilmiş)
                [5]: Hız Z bileşeni (normalize edilmiş)
                [6]: Rol (0=takipçi, 1=lider)
            - edge_index: Graf bağlantıları (iletişim menzili ve SNR bazlı)
            - y: GAT kodları (0=OK, 1=ENGEL, 2=CARPISMA, 3=KOPUK, 4=UZAK)
        """
        # Senaryo modülünü import et
        instance = self._get_senaryo_instance()
        if instance is None:
            # Fallback: ortam.veri_uret kullan
            return veri_uret(n_rovs=n_rovs)
        
        # Rastgele parametreler (eğer belirtilmemişse)
        # ROV sayısı 4-9 arasında - HER EPOCH'TA RASTGELE
        if n_rovs is None:
            n_rovs = np.random.randint(4, 10)  # 4-9 arası (10 dahil değil)
        if n_engels is None:
            n_engels = np.random.randint(10, 25)
        if havuz_genisligi is None:
            havuz_genisligi = 200.0
        
        # Ortam oluşturma kontrolü: Sadece her 25 epoch'ta bir yeni ortam oluştur
        yeni_ortam_olustur = False
        
        if not instance.aktif:
            # İlk kez oluştur
            yeni_ortam_olustur = True
        else:
            # Her 25 epoch'ta bir yeni ortam oluştur (ROV sayısı fark etmeksizin)
            self._ortam_olusturma_sayaci += 1
            if self._ortam_olusturma_sayaci % 100 == 0:
                yeni_ortam_olustur = True
        
        if yeni_ortam_olustur:
            # Yeni ortam oluştur (ROV sayısı ve engel sayısı rastgele)
            instance.uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi, verbose=False)
            # Sayaç sıfırla (yeni ortam oluşturulduğunda)
            self._ortam_olusturma_sayaci = 0
            # Yeni ortam oluşturulduğunda birkaç adım simülasyon çalıştır (fizik ve sensör güncellemeleri için)
            for _ in range(5):
                instance.guncelle(0.016)
        else:
            # Sadece pozisyonları yeniden dağıt (mevcut ROV ve engel sayısını koru)
            # Simülasyon adımlarına gerek yok - sadece pozisyonlar değişiyor
            instance._nesneleri_yeniden_dagit()
        
        # ROV verilerini topla
        n_rovs_gercek = len(instance.ortam.rovs)
        if n_rovs_gercek == 0:
            # Fallback: ortam.veri_uret kullan
            return veri_uret(n_rovs=n_rovs)
        
        # Özellik matrisi (7 özellik: GAT_kodu, batarya, SNR, derinlik, vx, vz, rol)
        x = torch.zeros((n_rovs_gercek, 7), dtype=torch.float)
        
        # Pozisyon matrisi (mesafe hesaplamaları için)
        positions = []
        danger_map = {}  # {rov_id: gat_kodu}
        
        # Her ROV için veri topla
        for i in range(n_rovs_gercek):
            # GPS koordinatları
            gps = instance.get(i, 'gps')
            if gps is None:
                gps = np.array([0.0, 0.0, -5.0])
            elif isinstance(gps, (list, tuple)):
                gps = np.array(gps)
            positions.append(gps[:2])  # Sadece x, y (2D düzlem)
            
            # Batarya
            battery = instance.get(i, 'batarya')
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
            velocity = instance.get(i, 'hiz')
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
            role = instance.get(i, 'rol')
            if role is None:
                role = 0
            x[i][6] = 1.0 if role == 1 else 0.0
        
        # Mesafe matrisi
        dist_matrix = np.zeros((n_rovs_gercek, n_rovs_gercek))
        for i in range(n_rovs_gercek):
            for j in range(n_rovs_gercek):
                if i != j:
                    dist_matrix[i, j] = np.linalg.norm(np.array(positions[i]) - np.array(positions[j]))
        
        # GAT kodlarını hesapla (gerçek durumlara göre)
        for i in range(n_rovs_gercek):
            code = 0
            
            # 1. Çarpışma kontrolü (en yüksek öncelik)
            min_rov_dist = np.min([dist_matrix[i, j] for j in range(n_rovs_gercek) if j != i])
            if min_rov_dist < GATLimitleri.CARPISMA:
                code = 2
                danger_map[i] = code
            
            # 2. Engel yakınlığı kontrolü (sonar verisi)
            if code == 0:
                sonar = instance.get(i, 'sonar')
                if sonar is not None and sonar > 0 and sonar < GATLimitleri.ENGEL:
                    code = 1
                    danger_map[i] = code
            
            # 3. Bağlantı kopması kontrolü
            if code == 0:
                if min_rov_dist > GATLimitleri.KOPMA:
                    code = 3
                    danger_map[i] = code
            
            # 4. Liderden uzaklık kontrolü (sadece takipçiler için)
            if code == 0 and i != 0:
                lider_dist = dist_matrix[i, 0]
                if lider_dist > GATLimitleri.UZAK:
                    code = 4  # GAT kodu 4 = UZAK
                    danger_map[i] = code
            
            # GAT kodu özelliği (normalize edilmiş)
            x[i][0] = float(code / 5.0)
        
        # Graf bağlantıları (iletişim menzili bazlı)
        sources, targets = [], []
        iletişim_menzili = GATLimitleri.KOPMA
        
        for i in range(n_rovs_gercek):
            for j in range(n_rovs_gercek):
                if i != j and dist_matrix[i, j] < iletişim_menzili:
                    # SNR bazlı bağlantı olasılığı
                    snr_i = x[i][2].item()
                    snr_j = x[j][2].item()
                    connection_prob = (snr_i + snr_j) / 2.0
                    
                    if np.random.random() < connection_prob:
                        sources.append(i)
                        targets.append(j)
        
        edge_index = torch.tensor([sources, targets], dtype=torch.long) if sources else torch.zeros((2, 0), dtype=torch.long)
        
        # Hedef etiketler (Y) - Graph yayılımı ile
        y = torch.zeros(n_rovs_gercek, dtype=torch.long)
        G = nx.Graph()
        G.add_nodes_from(range(n_rovs_gercek))
        if len(sources) > 0:
            G.add_edges_from(zip(sources, targets))
        
        # Öncelik sırası: Çarpışma > Engel > Kopma > Uzak > OK
        priority = {2: 0, 1: 1, 3: 2, 4: 3, 0: 4}
        
        for i in range(n_rovs_gercek):
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
    
    def veri_uret_batch(self, batch_size=32, n_rovs=None, n_engels=None, havuz_genisligi=None):
        """
        Toplu veri üretimi (batch training için).
        
        Args:
            batch_size (int): Batch boyutu
            n_rovs (int, optional): ROV sayısı
            n_engels (int, optional): Engel sayısı
            havuz_genisligi (float, optional): Havuz genişliği
        
        Returns:
            list: Data objeleri listesi
        """
        batch = []
        for _ in range(batch_size):
            data = self.veri_uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi)
            batch.append(data)
        return batch
    
    def istatistikler(self, n_samples=100, n_rovs=None, n_engels=None):
        """
        Veri dağılımı istatistiklerini hesaplar.
        
        Args:
            n_samples (int): Örnek sayısı
            n_rovs (int, optional): ROV sayısı
            n_engels (int, optional): Engel sayısı
        
        Returns:
            dict: İstatistikler (GAT kodları dağılımı, ortalama özellikler, vb.)
        """
        gat_kodlari = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
        toplam_rov = 0
        toplam_edge = 0
        
        for _ in range(n_samples):
            data = self.veri_uret(n_rovs=n_rovs, n_engels=n_engels)
            toplam_rov += data.x.shape[0]
            toplam_edge += data.edge_index.shape[1]
            
            for code in data.y.numpy():
                if code in gat_kodlari:
                    gat_kodlari[code] += 1
        
        return {
            'toplam_ornek': n_samples,
            'toplam_rov': toplam_rov,
            'ortalama_rov_per_ornek': toplam_rov / n_samples,
            'toplam_edge': toplam_edge,
            'ortalama_edge_per_ornek': toplam_edge / n_samples,
            'gat_kodlari_dagilimi': {k: v / toplam_rov for k, v in gat_kodlari.items()},
            'gat_kodlari_sayilari': gat_kodlari
        }


# Global instance (kolay kullanım için)
_veri_uretici = GATVeriUretici()


def veri_uret_gat(n_rovs=None, n_engels=None, havuz_genisligi=None):
    """
    GAT eğitimi için veri üretir (global fonksiyon wrapper).
    
    Args:
        n_rovs (int, optional): ROV sayısı
        n_engels (int, optional): Engel sayısı
        havuz_genisligi (float, optional): Havuz genişliği
    
    Returns:
        torch_geometric.data.Data: GAT modeli için hazırlanmış veri
    """
    return _veri_uretici.veri_uret(n_rovs=n_rovs, n_engels=n_engels, havuz_genisligi=havuz_genisligi)


class GAT_Modeli(torch.nn.Module):
    def __init__(self, hidden_channels=16, num_heads=4, dropout=0.1):
        """
        GAT Modeli - Optimize edilebilir hiperparametrelerle.
        
        Args:
            hidden_channels (int): Gizli katman boyutu
            num_heads (int): Attention head sayısı
            dropout (float): Dropout oranı
        """
        super().__init__()
        # Giriş: 7 Özellik
        self.conv1 = GATConv(in_channels=7, out_channels=hidden_channels, heads=num_heads, dropout=dropout)
        # Çıkış: 6 Sınıf
        self.conv2 = GATConv(hidden_channels * num_heads, 6, heads=1, dropout=dropout)
        self.dropout = dropout
        
        # Otomatik Yükleme
        if os.path.exists(MODEL_DOSYA_ADI):
            try:
                device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
                self.load_state_dict(torch.load(MODEL_DOSYA_ADI, map_location=device))
            except: pass

    def forward(self, x, edge_index, return_attention=False):
        x = self.conv1(x, edge_index)
        x = F.elu(x)
        x = F.dropout(x, p=self.dropout, training=self.training)
        
        if return_attention:
            x, (ei, alpha) = self.conv2(x, edge_index, return_attention_weights=True)
            return F.log_softmax(x, dim=1), ei, alpha
        else:
            x = self.conv2(x, edge_index)
            return F.log_softmax(x, dim=1)


# --- BURAYA TAŞINDI ---
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
        with torch.no_grad():
            out, edge_idx, alpha = self.model(veri.x, veri.edge_index, return_attention=True)
            tahminler = out.argmax(dim=1).numpy()
        return tahminler, edge_idx, alpha

def Train(veri_kaynagi=None, epochs=5000, lr=0.001, hidden_channels=16, num_heads=4, 
          dropout=0.1, weight_decay=1e-4, use_senaryo=True):
    """
    Geliştirilmiş Eğitim Fonksiyonu - Senaryo verileri ve hiperparametre optimizasyonu ile.
    
    Args:
        veri_kaynagi: Veri kaynağı (None, callable, veya Data objesi)
        epochs (int): Eğitim epoch sayısı
        lr (float): Learning rate
        hidden_channels (int): Gizli katman boyutu
        num_heads (int): Attention head sayısı
        dropout (float): Dropout oranı
        weight_decay (float): Weight decay (L2 regularization)
        use_senaryo (bool): Senaryo verilerini kullan (True) veya ortam.veri_uret (False)
    """
    print(f"🚀 GAT Eğitimi Başlıyor (Senaryo Modu: {use_senaryo})... ({epochs} Adım)")
    print(f"   📊 Hiperparametreler: hidden={hidden_channels}, heads={num_heads}, dropout={dropout:.2f}, lr={lr:.4f}")
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = GAT_Modeli(hidden_channels=hidden_channels, num_heads=num_heads, dropout=dropout).to(device)
    model.train()
    
    # Optimizer ve Scheduler
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=500, min_lr=1e-6)
    criterion = nn.NLLLoss()
    
    best_loss = float('inf')
    loss_history = []
    
    # Senaryo verilerini kullan
    if use_senaryo:
        try:
            # GATVeriUretici sınıfını kullan
            veri_uretici = GATVeriUretici()
            veri_kaynagi = veri_uretici.veri_uret
            print("   ✅ Senaryo veri üretimi aktif (gerçek simülasyon verileri)")
        except Exception as e:
            print(f"   ⚠️ GATVeriUretici yüklenemedi, ortam.veri_uret kullanılıyor: {e}")
            use_senaryo = False
    
    print()
    print("=" * 80)
    print("EĞİTİM BAŞLIYOR")
    print("=" * 80)
    print()
    
    for epoch in range(1, epochs + 1):
        # Veri Yönetimi
        if veri_kaynagi is None:
            if use_senaryo:
                try:
                    data = veri_uret_gat()
                    if epoch == 1 or epoch % 100 == 0:
                        # Ada sayısını al (veri üretildikten sonra)
                        ada_sayisi = 0
                        try:
                            from . import senaryo
                            instance = senaryo._get_instance()
                            if instance and hasattr(instance, 'ortam') and hasattr(instance.ortam, 'island_positions'):
                                ada_sayisi = len(instance.ortam.island_positions) if instance.ortam.island_positions else 0
                        except Exception as e:
                            # Hata olsa bile 0 göster
                            ada_sayisi = 0
                        print(f"   📊 Epoch {epoch}: Veri üretildi - {data.x.shape[0]} ROV, {data.edge_index.shape[1]} edge, {ada_sayisi} Ada")
                except Exception as e:
                    print(f"   ⚠️ Epoch {epoch}: Senaryo veri üretimi hatası, ortam.veri_uret kullanılıyor: {e}")
                    data = veri_uret()
            else:
                data = veri_uret()
        elif callable(veri_kaynagi):
            data = veri_kaynagi()
            # Ada sayısını her zaman al (hata olsa bile göster)
            ada_sayisi = 0
            try:
                from . import senaryo
                instance = senaryo._get_instance()
                if instance and hasattr(instance, 'ortam') and hasattr(instance.ortam, 'island_positions'):
                    ada_sayisi = len(instance.ortam.island_positions) if instance.ortam.island_positions else 0
            except Exception as e:
                # Hata olsa bile 0 göster
                ada_sayisi = 0
            
            if epoch == 1 or epoch % 100 == 0:
                print(f"   📊 Epoch {epoch}: Veri üretildi - {data.x.shape[0]} ROV, {data.edge_index.shape[1]} edge, {ada_sayisi} Ada")
        else:
            data = veri_kaynagi
        
        data = data.to(device)
        
        optimizer.zero_grad()
        out = model(data.x, data.edge_index)
        loss = criterion(out, data.y)
        
        loss.backward()
        
        # Gradient Clipping
        grad_norm = torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        
        optimizer.step()
        
        # Loss takibi
        loss_history.append(loss.item())
        if len(loss_history) > 100:
            loss_history.pop(0)
        avg_loss = sum(loss_history) / len(loss_history)
        
        # Scheduler güncelle
        scheduler.step(avg_loss)
        
        # En İyi Modeli Kaydet
        model_kaydedildi = False
        if avg_loss < best_loss and epoch > 100:
            best_loss = avg_loss
            torch.save(model.state_dict(), MODEL_DOSYA_ADI)
            model_kaydedildi = True
        
        # Detaylı Raporlama
        lr_curr = optimizer.param_groups[0]['lr']
        
        # Doğruluk ve sınıf dağılımı hesapla
        with torch.no_grad():
            pred = out.argmax(dim=1)
            accuracy = (pred == data.y).float().mean().item()
            
            # GAT kodları dağılımı
            y_unique, y_counts = torch.unique(data.y, return_counts=True)
            y_dist = {int(k): int(v) for k, v in zip(y_unique, y_counts)}
            
            # Tahmin dağılımı
            pred_unique, pred_counts = torch.unique(pred, return_counts=True)
            pred_dist = {int(k): int(v) for k, v in zip(pred_unique, pred_counts)}
        
        # Her epoch'ta detaylı log
        if epoch == 1:
            print(f"   🔹 Epoch {epoch:4d}/{epochs} | Loss: {loss.item():.4f} | Ort. Loss: {avg_loss:.4f} | Acc: {accuracy:.2%} | LR: {lr_curr:.6f} | Grad: {grad_norm:.3f}")
            print(f"      📈 Gerçek GAT: {y_dist}")
            print(f"      🎯 Tahmin GAT: {pred_dist}")
        elif epoch % 30 == 0:
            # Her 30 epoch'ta detaylı bilgi
            print(f"   🔹 Epoch {epoch:4d}/{epochs} | Loss: {loss.item():.4f} | Ort. Loss: {avg_loss:.4f} | Acc: {accuracy:.2%} | LR: {lr_curr:.6f} | Grad: {grad_norm:.3f}")
            print(f"      📈 Gerçek GAT: {y_dist}")
            print(f"      🎯 Tahmin GAT: {pred_dist}")
            if model_kaydedildi:
                print(f"      ✅ Yeni en iyi model kaydedildi! (Loss: {best_loss:.4f})")
        elif epoch % 10 == 0:
            print(f"   🔹 Epoch {epoch:4d}/{epochs} | Loss: {loss.item():.4f} | Ort. Loss: {avg_loss:.4f} | Acc: {accuracy:.2%} | LR: {lr_curr:.6f}")
        else:
            # Her epoch'ta kısa log
            print(f"   Epoch {epoch:4d}/{epochs} | Loss: {loss.item():.4f} | Acc: {accuracy:.2%}", end='\r')
    
    print()
    print("=" * 80)
    print("✅ EĞİTİM TAMAMLANDI")
    print("=" * 80)
    print(f"   Toplam epoch: {epochs}")
    print(f"   En düşük loss: {best_loss:.4f}")
    print(f"   Son loss: {loss_history[-1] if loss_history else 'N/A':.4f}")
    print(f"   Son ortalama loss: {avg_loss:.4f}")
    print(f"   Model dosyası: {MODEL_DOSYA_ADI}")
    print("=" * 80)
    print()
    return model, best_loss

def optimize_hyperparameters(n_trials=20, epochs_per_trial=1000):
    """
    Hiperparametre optimizasyonu (Optuna kullanarak).
    
    Args:
        n_trials (int): Optimizasyon deneme sayısı
        epochs_per_trial (int): Her deneme için epoch sayısı
    
    Returns:
        dict: En iyi hiperparametreler
    """
    try:
        import optuna
    except ImportError:
        print("❌ Optuna bulunamadı. Yüklemek için: pip install optuna")
        print("   Grid search kullanılıyor...")
        return optimize_hyperparameters_grid(epochs_per_trial=epochs_per_trial)
    
    print(f"🔍 Hiperparametre Optimizasyonu Başlıyor ({n_trials} deneme)...")
    
    def objective(trial):
        # Hiperparametre önerileri
        lr = trial.suggest_loguniform('lr', 1e-4, 1e-2)
        hidden_channels = trial.suggest_int('hidden_channels', 8, 32, step=4)
        num_heads = trial.suggest_int('num_heads', 2, 8, step=2)
        dropout = trial.suggest_uniform('dropout', 0.0, 0.3)
        weight_decay = trial.suggest_loguniform('weight_decay', 1e-5, 1e-3)
        
        # Model eğit
        _, best_loss = Train(
            epochs=epochs_per_trial,
            lr=lr,
            hidden_channels=hidden_channels,
            num_heads=num_heads,
            dropout=dropout,
            weight_decay=weight_decay,
            use_senaryo=True
        )
        
        return best_loss
    
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=n_trials, show_progress_bar=True)
    
    print("\n✅ Optimizasyon Tamamlandı!")
    print(f"   En iyi loss: {study.best_value:.4f}")
    print(f"   En iyi hiperparametreler:")
    for key, value in study.best_params.items():
        print(f"      {key}: {value}")
    
    # En iyi parametrelerle final eğitim
    print("\n🎯 En iyi parametrelerle final eğitim başlıyor...")
    Train(
        epochs=epochs_per_trial * 3,  # Daha uzun eğitim
        **study.best_params,
        use_senaryo=True
    )
    
    return study.best_params


def optimize_hyperparameters_grid(epochs_per_trial=1000):
    """
    Grid search ile hiperparametre optimizasyonu (Optuna yoksa).
    """
    print("🔍 Grid Search Hiperparametre Optimizasyonu...")
    
    best_params = None
    best_loss = float('inf')
    
    # Grid parametreleri
    lrs = [0.001, 0.002, 0.005]
    hidden_channels_list = [12, 16, 20, 24]
    num_heads_list = [2, 4, 6]
    dropouts = [0.0, 0.1, 0.2]
    weight_decays = [1e-4, 5e-4, 1e-3]
    
    total_trials = len(lrs) * len(hidden_channels_list) * len(num_heads_list) * len(dropouts) * len(weight_decays)
    current_trial = 0
    
    for lr in lrs:
        for hidden_channels in hidden_channels_list:
            for num_heads in num_heads_list:
                for dropout in dropouts:
                    for weight_decay in weight_decays:
                        current_trial += 1
                        print(f"\n[{current_trial}/{total_trials}] Test ediliyor: lr={lr}, hidden={hidden_channels}, heads={num_heads}, dropout={dropout:.2f}, wd={weight_decay}")
                        
                        _, trial_loss = Train(
                            epochs=epochs_per_trial,
                            lr=lr,
                            hidden_channels=hidden_channels,
                            num_heads=num_heads,
                            dropout=dropout,
                            weight_decay=weight_decay,
                            use_senaryo=True
                        )
                        
                        if trial_loss < best_loss:
                            best_loss = trial_loss
                            best_params = {
                                'lr': lr,
                                'hidden_channels': hidden_channels,
                                'num_heads': num_heads,
                                'dropout': dropout,
                                'weight_decay': weight_decay
                            }
                            print(f"   ✅ Yeni en iyi! Loss: {best_loss:.4f}")
    
    print(f"\n✅ Grid Search Tamamlandı!")
    print(f"   En iyi loss: {best_loss:.4f}")
    print(f"   En iyi hiperparametreler: {best_params}")
    
    # En iyi parametrelerle final eğitim
    if best_params:
        print("\n🎯 En iyi parametrelerle final eğitim başlıyor...")
        Train(epochs=epochs_per_trial * 3, **best_params, use_senaryo=True)
    
    return best_params


def reset():
    if os.path.exists(MODEL_DOSYA_ADI):
        os.remove(MODEL_DOSYA_ADI)
        print(f"♻️  SIFIRLANDI: {MODEL_DOSYA_ADI}")
