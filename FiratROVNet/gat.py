import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch_geometric.nn import GATConv
import os
from .ortam import veri_uret

MODEL_DOSYA_ADI = "rov_modeli_multi.pth"

class GAT_Modeli(torch.nn.Module):
    def __init__(self):
        super().__init__()
        # Giriş: 7 Özellik
        self.conv1 = GATConv(in_channels=7, out_channels=16, heads=4)
        # Çıkış: 6 Sınıf
        self.conv2 = GATConv(64, 6, heads=1)
        
        # Otomatik Yükleme
        if os.path.exists(MODEL_DOSYA_ADI):
            try:
                device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
                self.load_state_dict(torch.load(MODEL_DOSYA_ADI, map_location=device))
            except: pass

    def forward(self, x, edge_index, return_attention=False):
        x = self.conv1(x, edge_index)
        x = F.elu(x)
        
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

def Train(veri_kaynagi=None, epochs=5000, lr=0.001):
    """
    Geliştirilmiş Eğitim Fonksiyonu (Hata Düzeltildi)
    """
    print(f"🚀 GAT Eğitimi Başlıyor (Akıllı Mod)... ({epochs} Adım)")
    
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model = GAT_Modeli().to(device)
    model.train()
    
    # 1. Optimizer ve Scheduler (verbose kaldırıldı)
    optimizer = optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
    # HATA DÜZELTİLDİ: verbose=True parametresi kaldırıldı
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=1000)
    criterion = nn.NLLLoss()
    
    best_loss = float('inf')
    loss_history = [] 
    
    for epoch in range(1, epochs + 1):
        # Veri Yönetimi
        if veri_kaynagi is None: data = veri_uret()
        elif callable(veri_kaynagi): data = veri_kaynagi()
        else: data = veri_kaynagi
        
        data = data.to(device)
        
        optimizer.zero_grad()
        out = model(data.x, data.edge_index)
        loss = criterion(out, data.y)
        
        loss.backward()
        
        # 2. Gradient Clipping
        torch.nn.utils.clip_grad_norm_(model.parameters(), max_norm=1.0)
        
        optimizer.step()
        
        # Loss takibi
        loss_history.append(loss.item())
        if len(loss_history) > 100: loss_history.pop(0)
        avg_loss = sum(loss_history) / len(loss_history)
        
        # Scheduler güncelle
        scheduler.step(avg_loss)
        
        # 3. En İyi Modeli Kaydet
        if avg_loss < best_loss and epoch > 100:
            best_loss = avg_loss
            torch.save(model.state_dict(), MODEL_DOSYA_ADI)
        
        # Raporlama
        if epoch % 500 == 0:
            lr_curr = optimizer.param_groups[0]['lr']
            print(f"   🔹 Ders {epoch}/{epochs} | Anlık Loss: {loss.item():.4f} | Ort. Loss: {avg_loss:.4f} | LR: {lr_curr:.6f}")
            
    print(f"✅ Eğitim Tamamlandı! En düşük hata (Loss): {best_loss:.4f}")

def reset():
    if os.path.exists(MODEL_DOSYA_ADI):
        os.remove(MODEL_DOSYA_ADI)
        print(f"♻️  SIFIRLANDI: {MODEL_DOSYA_ADI}")
