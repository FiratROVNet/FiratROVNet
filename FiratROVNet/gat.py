import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch_geometric.nn import GATConv
import os
from .ortam import veri_uret

MODEL_DOSYA_ADI = "rov_modeli_multi.pth"

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
            from . import senaryo
            veri_kaynagi = senaryo.veri_uret_gat
            print("   ✅ Senaryo veri üretimi aktif")
        except Exception as e:
            print(f"   ⚠️ Senaryo yüklenemedi, ortam.veri_uret kullanılıyor: {e}")
            use_senaryo = False
    
    for epoch in range(1, epochs + 1):
        # Veri Yönetimi
        if veri_kaynagi is None:
            if use_senaryo:
                try:
                    from . import senaryo
                    data = senaryo.veri_uret_gat()
                except:
                    data = veri_uret()
            else:
                data = veri_uret()
        elif callable(veri_kaynagi):
            data = veri_kaynagi()
        else:
            data = veri_kaynagi
        
        data = data.to(device)
        
        optimizer.zero_grad()
        out = model(data.x, data.edge_index)
        loss = criterion(out, data.y)
        
        loss.backward()
        
        # Gradient Clipping
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        
        optimizer.step()
        
        # Loss takibi
        loss_history.append(loss.item())
        if len(loss_history) > 100:
            loss_history.pop(0)
        avg_loss = sum(loss_history) / len(loss_history)
        
        # Scheduler güncelle
        scheduler.step(avg_loss)
        
        # En İyi Modeli Kaydet
        if avg_loss < best_loss and epoch > 100:
            best_loss = avg_loss
            torch.save(model.state_dict(), MODEL_DOSYA_ADI)
        
        # Raporlama
        if epoch % 500 == 0:
            lr_curr = optimizer.param_groups[0]['lr']
            # Doğruluk hesapla
            with torch.no_grad():
                pred = out.argmax(dim=1)
                accuracy = (pred == data.y).float().mean().item()
            print(f"   🔹 Epoch {epoch}/{epochs} | Loss: {loss.item():.4f} | Ort. Loss: {avg_loss:.4f} | Acc: {accuracy:.2%} | LR: {lr_curr:.6f}")
    
    print(f"✅ Eğitim Tamamlandı! En düşük hata (Loss): {best_loss:.4f}")
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
