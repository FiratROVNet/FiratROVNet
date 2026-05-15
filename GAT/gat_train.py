"""
GAT Eğitim Modülü
Filo sınıfından senaryo çekip GAT modelini eğitir.
"""

import sys
import os

# Proje kök dizinini path'e ekle
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch_geometric.nn import GATConv
import numpy as np
# Pencere açmadan sadece dosyaya çiz (beyaz sayfa flash'ı önler)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from FiratROVNet.gnc import Filo

MODEL_DOSYA_ADI = "rov_modeli_multi.pth"
GRAFIK_DOSYA_ADI = "gat_egitim_grafigi.png"


class FocalLoss(nn.Module):
    """
    Focal Loss - Nadir sınıflar için daha iyi öğrenme.
    """
    def __init__(self, alpha=1, gamma=2):
        super().__init__()
        self.alpha = alpha
        self.gamma = gamma
    
    def forward(self, inputs, targets):
        ce_loss = F.nll_loss(inputs, targets, reduction='none')
        pt = torch.exp(-ce_loss)
        focal_loss = self.alpha * (1 - pt) ** self.gamma * ce_loss
        return focal_loss.mean()


class GAT_Modeli(torch.nn.Module):
    def __init__(self, hidden_channels=16, num_heads=4, dropout=0.1, load_checkpoint=True):
        """
        GAT Modeli - Optimize edilebilir hiperparametrelerle.
        
        Args:
            hidden_channels (int): Gizli katman boyutu
            num_heads (int): Attention head sayısı
            dropout (float): Dropout oranı
            load_checkpoint (bool): True ise mevcut model dosyası varsa yükle (eğitime devam için)
        """
        super().__init__()
        # Giriş: 9 Özellik (mesafe bilgileri eklendi)
        self.conv1 = GATConv(in_channels=9, out_channels=hidden_channels, heads=num_heads, dropout=dropout)
        # Çıkış: 6 Sınıf
        self.conv2 = GATConv(hidden_channels * num_heads, 6, heads=1, dropout=dropout)
        self.dropout = dropout
        
        # İsteğe bağlı: Mevcut modeli yükle (resume için)
        if load_checkpoint and os.path.exists(MODEL_DOSYA_ADI):
            try:
                device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
                checkpoint = torch.load(MODEL_DOSYA_ADI, map_location=device)
                
                # Eski checkpoint kontrolü: 7 feature'dan 9 feature'a uyarlama
                if 'conv1.lin.weight' in checkpoint:
                    old_weight = checkpoint['conv1.lin.weight']
                    # Eski model 7 feature, yeni model 9 feature bekliyor
                    if old_weight.shape[1] == 7:
                        # Yeni model state_dict'ini al
                        new_state_dict = self.state_dict()
                        new_weight = new_state_dict['conv1.lin.weight']
                        
                        # Eski ağırlıkları kopyala (ilk 7 feature için)
                        new_weight[:, :7] = old_weight
                        
                        # Yeni feature'lar için küçük rastgele ağırlıklar ekle (Xavier initialization benzeri)
                        if new_weight.shape[1] > 7:
                            import torch.nn.init as init
                            init.xavier_uniform_(new_weight[:, 7:], gain=0.1)
                        
                        # Güncellenmiş ağırlığı checkpoint'e ekle
                        checkpoint['conv1.lin.weight'] = new_weight
                
                # Güncellenmiş checkpoint'i yükle
                self.load_state_dict(checkpoint, strict=False)
            except Exception:
                pass

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


def train(epochs=1000, lr=0.001, hidden_channels=16, num_heads=4, 
          dropout=0.1, weight_decay=1e-4, resume=True):
    """
    GAT modelini eğitir.
    
    Args:
        epochs (int): Eğitim epoch sayısı
        lr (float): Learning rate
        hidden_channels (int): Gizli katman boyutu
        num_heads (int): Attention head sayısı
        dropout (float): Dropout oranı
        weight_decay (float): Weight decay (L2 regularization)
        resume (bool): True ise mevcut model varsa yükleyip kaldığı yerden devam eder; False ise sıfırdan başlar
    
    Returns:
        tuple: (model, best_loss)
    """
    print(f"🚀 GAT Eğitimi Başlıyor... ({epochs} Adım)")
    print(f"   📊 Hiperparametreler: hidden={hidden_channels}, heads={num_heads}, dropout={dropout:.2f}, lr={lr:.4f}")
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = GAT_Modeli(
        hidden_channels=hidden_channels,
        num_heads=num_heads,
        dropout=dropout,
        load_checkpoint=resume
    ).to(device)
    if resume and os.path.exists(MODEL_DOSYA_ADI):
        print(f"   📂 Mevcut model yüklendi: {MODEL_DOSYA_ADI} — eğitime kaldığı yerden devam ediliyor.")
    model.train()
    
    # Optimizer ve Scheduler
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=500, min_lr=1e-6)
    
    # Class weights (nadir sınıfları daha fazla ağırlıklandır)
    # Sınıf frekanslarına göre ters orantılı ağırlıklar
    class_weights = torch.tensor([
        0.5,  # 0: OK (çoğunluk sınıfı)
        2.0,  # 1: Engel (nadir)
        3.0,  # 2: Çarpışma (çok nadir)
        1.0,  # 3: Kopma (orta)
        2.0,  # 4: Uzak (nadir)
        1.0   # 5: Kullanılmıyor
    ], dtype=torch.float32).to(device)
    
    # Focal Loss kullan (nadir sınıflar için daha iyi)
    # Alternatif: nn.NLLLoss(weight=class_weights) kullanılabilir
    criterion = FocalLoss(alpha=1, gamma=2)
    
    best_loss = float('inf')
    loss_history = []
    
    # Eğitim metriklerini saklamak için listeler
    epoch_losses = []
    epoch_accuracies = []
    epoch_numbers = []
    
    # Filo instance'ı oluştur
    filo = Filo()
    
    print()
    print("=" * 80)
    print("EĞİTİM BAŞLIYOR")
    print("=" * 80)
    print()
    
    for epoch in range(1, epochs + 1):
        # Senaryo verisi üret
        senaryo_verisi = filo.gat_veri_uret()
        if senaryo_verisi is None:
            print(f"   ⚠️ Epoch {epoch}: Senaryo verisi üretilemedi, atlanıyor...")
            continue
        
        # GAT verisini oluştur
        try:
            from GAT.gat_kod_hesapla import tam_veri_olustur
            data = tam_veri_olustur(senaryo_verisi)
        except Exception as e:
            print(f"   ⚠️ Epoch {epoch}: GAT verisi oluşturulamadı: {e}")
            continue
        
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
        
        # Detaylı Raporlama
        lr_curr = optimizer.param_groups[0]['lr']
        
        # Doğruluk ve sınıf dağılımı hesapla
        with torch.no_grad():
            pred = out.argmax(dim=1)
            accuracy = (pred == data.y).float().mean().item()
        
        # Metrikleri kaydet (accuracy'den SONRA, böylece uzunluklar eşleşir)
        epoch_losses.append(loss.item())
        epoch_numbers.append(epoch)
        epoch_accuracies.append(accuracy)
        
        # En İyi Modeli Kaydet (grafik sadece eğitim sonunda kaydedilir - flash/yavaşlama önlenir)
        model_kaydedildi = False
        if avg_loss < best_loss and epoch > 100:
            best_loss = avg_loss
            torch.save(model.state_dict(), MODEL_DOSYA_ADI)
            model_kaydedildi = True
            
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
    
    # Eğitim sonunda grafiği kaydet
    _kaydet_grafik(epoch_numbers, epoch_losses, epoch_accuracies, GRAFIK_DOSYA_ADI)
    print(f"   📊 Grafik dosyası: {GRAFIK_DOSYA_ADI}")
    print("=" * 80)
    print()
    return model, best_loss


def _kaydet_grafik(epoch_numbers, losses, accuracies, dosya_adi):
    """
    Loss ve Accuracy grafiklerini kaydeder.
    
    Args:
        epoch_numbers (list): Epoch numaraları
        losses (list): Loss değerleri
        accuracies (list): Accuracy değerleri
        dosya_adi (str): Kaydedilecek dosya adı
    """
    if not epoch_numbers or not losses or not accuracies:
        return
    
    # Grafik oluştur
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
    
    # Loss grafiği
    ax1.plot(epoch_numbers, losses, 'b-', linewidth=1.5, alpha=0.7, label='Loss')
    ax1.set_xlabel('Epoch', fontsize=12)
    ax1.set_ylabel('Loss', fontsize=12)
    ax1.set_title('GAT Eğitim Loss Grafiği', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Moving average (son 50 epoch'un ortalaması)
    if len(losses) > 50:
        window_size = min(50, len(losses) // 10)
        moving_avg = []
        for i in range(len(losses)):
            start = max(0, i - window_size + 1)
            moving_avg.append(np.mean(losses[start:i+1]))
        ax1.plot(epoch_numbers, moving_avg, 'r-', linewidth=2, alpha=0.8, label=f'Moving Avg ({window_size})')
        ax1.legend()
    
    # Accuracy grafiği
    ax2.plot(epoch_numbers, accuracies, 'g-', linewidth=1.5, alpha=0.7, label='Accuracy')
    ax2.set_xlabel('Epoch', fontsize=12)
    ax2.set_ylabel('Accuracy', fontsize=12)
    ax2.set_title('GAT Eğitim Accuracy Grafiği', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_ylim([0, 1.05])  # Accuracy 0-1 arası
    
    # Moving average (son 50 epoch'un ortalaması)
    if len(accuracies) > 50:
        window_size = min(50, len(accuracies) // 10)
        moving_avg_acc = []
        for i in range(len(accuracies)):
            start = max(0, i - window_size + 1)
            moving_avg_acc.append(np.mean(accuracies[start:i+1]))
        ax2.plot(epoch_numbers, moving_avg_acc, 'orange', linewidth=2, alpha=0.8, label=f'Moving Avg ({window_size})')
        ax2.legend()
    
    # İstatistikler ekle
    if losses:
        min_loss = min(losses)
        min_loss_epoch = epoch_numbers[losses.index(min_loss)]
        max_acc = max(accuracies)
        max_acc_epoch = epoch_numbers[accuracies.index(max_acc)]
        
        stats_text = f"Min Loss: {min_loss:.4f} (Epoch {min_loss_epoch})\n"
        stats_text += f"Max Accuracy: {max_acc:.2%} (Epoch {max_acc_epoch})\n"
        stats_text += f"Final Loss: {losses[-1]:.4f}\n"
        stats_text += f"Final Accuracy: {accuracies[-1]:.2%}"
        
        fig.text(0.02, 0.02, stats_text, fontsize=10, 
                verticalalignment='bottom', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig(dosya_adi, dpi=300, bbox_inches='tight')
    plt.close()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="GAT model eğitimi")
    parser.add_argument("--epochs", type=int, default=200, help="Epoch sayısı (varsayılan: 200)")
    parser.add_argument("--no-resume", action="store_true", help="Mevcut modeli yükleme, sıfırdan başla")
    args = parser.parse_args()
    # Eğitimi başlat (resume=True: mevcut model varsa kaldığı yerden devam)
    model, best_loss = train(
        epochs=args.epochs,
        lr=0.001,
        hidden_channels=32,
        num_heads=4,
        dropout=0.1,
        resume=not args.no_resume,
    )
