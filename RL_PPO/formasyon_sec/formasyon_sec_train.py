import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from FiratROVNet.gnc import Filo
import numpy as np


def train_formasyon_secim():
    # Windows DLL hatası için torch import'unu buraya taşı
    import platform
    
    if platform.system() == 'Windows':
        try:
            import torch
            import torch.nn as nn
            import torch.optim as optim
        except OSError as e:
            if "WinError 1114" in str(e) or "DLL" in str(e):
                print("❌ PyTorch DLL hatası tespit edildi!")
                print("   Lütfen README_WINDOWS_FIX.md dosyasındaki çözümleri uygulayın.")
                print("   Hızlı çözüm: pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu")
                print("   Visual C++ Redistributables kurulumu gerekebilir.")
                return
            raise
    else:
        # Linux'ta normal import
        import torch
        import torch.nn as nn
        import torch.optim as optim
    
    # Model import'unu da buraya taşı (torch import'undan sonra)
    from formasyon_sec_rl_model import FormasyonSecimAgi
    
    # 1. Kurulum
    filo = Filo()
    model = FormasyonSecimAgi(input_dim=230, num_formations=20)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # Kayıp Fonksiyonları
    criterion_formation_id = nn.CrossEntropyLoss()  # Sınıflandırma için
    criterion_spacing = nn.MSELoss()                 # Aralık regresyonu
    criterion_yaw = nn.MSELoss()                     # Yaw regresyonu
    criterion_leader_pos = nn.MSELoss()              # Lider konum regresyonu

    # RL Metrikleri için değişkenler
    episode_rewards = []  # Her episode için reward
    avg_reward_window = 100  # Son N episode için ortalama
    policy_losses = []  # Policy loss'ları
    value_losses = []  # Value loss'ları
    entropy_losses = []  # Entropy loss'ları

    print("🚀 Formasyon Seçim Eğitimi Başlıyor...")

    for epoch in range(1000):  # 1000 iterasyon
        # Veri üret
        data = filo.uret_rl_egitim_verisi()
        if data is None:
            if epoch % 50 == 0:
                print("⚠️ [Eğitim] Veri üretimi başarısız (data=None)")
            continue
        
        # Giriş verilerini hazırla
        # Giriş: lider_pozisyon (3,) + lider_yaw (1,) + rov_filo_gps (24,) + hull_merkez (2,) + hull_noktalar (200,) = 230 boyut
        lider_pos = torch.FloatTensor(data["lider_pozisyon"])  # (3,)
        lider_yaw = torch.FloatTensor([data["lider_yaw"]])      # (1,)
        rov_filo_gps = torch.FloatTensor(data["rov_filo_gps"].flatten())  # (24,)
        hull_merkez = torch.FloatTensor(data["hull_merkez"])     # (2,)
        hull_noktalar = torch.FloatTensor(data["hull_noktalar"].flatten()[:200])  # (200,)
        
        # Giriş vektörünü birleştir (230 boyut)
        state = torch.cat([
            lider_pos,
            lider_yaw,
            rov_filo_gps,
            hull_merkez,
            hull_noktalar
        ]).unsqueeze(0)  # Batch boyutu ekle
        
        # Çıkış bilgilerini al (formasyon_sec() çıktısı)
        output_info = data["output"]
        if output_info is None:
            if epoch % 50 == 0:
                print("⚠️ [Eğitim] Çıkış bilgisi yok (output=None)")
            continue
        
        # Çıkış: (formasyon_id, aralik, yaw, koordinat)
        if isinstance(output_info, tuple) and len(output_info) >= 3:
            target_formation_id = torch.LongTensor([int(output_info[0]) if output_info[0] is not None else 0])
            target_spacing = torch.FloatTensor([[float(output_info[1]) if output_info[1] is not None else 30.0]])
            target_yaw = torch.FloatTensor([[float(output_info[2]) if output_info[2] is not None else 0.0]])
            # Lider konumu hedefe ekle (modele sağlanan lider konumunun aynısı olmalı)
            target_leader_pos = torch.FloatTensor([[lider_pos[0].item(), lider_pos[1].item(), lider_pos[2].item()]])
        else:
            continue

        # Forward Pass
        optimizer.zero_grad()
        formation_id_logits, spacing_pred, yaw_pred, leader_pos_pred = model(state)

        # Loss Calculation
        loss_formation_id = criterion_formation_id(formation_id_logits, target_formation_id)
        loss_spacing = criterion_spacing(spacing_pred, target_spacing)
        loss_yaw = criterion_yaw(yaw_pred, target_yaw)
        loss_leader_pos = criterion_leader_pos(leader_pos_pred, target_leader_pos)

        # Toplam Kayıp (Ağırlıklı)
        total_loss = loss_formation_id + 0.5 * loss_spacing + 0.3 * loss_yaw + 0.4 * loss_leader_pos

        # RL Metrikleri Hesaplama
        with torch.no_grad():
            # Policy Loss (Formation ID için CrossEntropyLoss)
            policy_loss = loss_formation_id.item()
            
            # Value Loss (Spacing, Yaw, Position için MSELoss'ların toplamı)
            value_loss = (0.5 * loss_spacing.item() + 0.3 * loss_yaw.item() + 0.4 * loss_leader_pos.item())
            
            # Entropy Loss (Logits'lerden entropy hesapla)
            probs = F.softmax(formation_id_logits, dim=1)
            log_probs = F.log_softmax(formation_id_logits, dim=1)
            entropy = -(probs * log_probs).sum(dim=1).mean()
            entropy_loss = entropy.item()
            
            # Reward Hesaplama (Accuracy ve loss'lara göre)
            pred_id = torch.argmax(formation_id_logits, dim=1)
            accuracy = (pred_id == target_formation_id).float().mean().item()
            # Reward: Accuracy yüksek, loss düşük olmalı
            # Reward = accuracy * 100 - total_loss (normalize edilmiş)
            episode_reward = accuracy * 100.0 - total_loss.item() * 5.0
            
            # Metrikleri kaydet
            episode_rewards.append(episode_reward)
            policy_losses.append(policy_loss)
            value_losses.append(value_loss)
            entropy_losses.append(entropy_loss)
            
            # Ortalama reward (son N episode)
            avg_reward = sum(episode_rewards[-avg_reward_window:]) / min(len(episode_rewards), avg_reward_window)

        # Backward Pass
        total_loss.backward()
        optimizer.step()

        if epoch % 50 == 0:
            print(
                "Epoch: {epoch} | Total: {loss:.6f} | Loss(id): {loss_id:.6f} | "
                "Loss(spacing): {loss_spacing:.6f} | Loss(yaw): {loss_yaw:.6f} | Loss(pos): {loss_pos:.6f} | "
                "Acc: {acc:.4f} | Pred Spacing: {pred_spacing:.2f} | Pred Pos: ({pos_x:.1f}, {pos_y:.1f}, {pos_z:.1f}) | "
                "Reward: {reward:.2f} | Avg Reward: {avg_reward:.2f} | "
                "Policy Loss: {policy_loss:.6f} | Value Loss: {value_loss:.6f} | Entropy: {entropy:.6f}".format(
                    epoch=epoch,
                    loss=total_loss.item(),
                    loss_id=loss_formation_id.item(),
                    loss_spacing=loss_spacing.item(),
                    loss_yaw=loss_yaw.item(),
                    loss_pos=loss_leader_pos.item(),
                    acc=accuracy,
                    pred_spacing=spacing_pred.item(),
                    pos_x=leader_pos_pred[0, 0].item(),
                    pos_y=leader_pos_pred[0, 1].item(),
                    pos_z=leader_pos_pred[0, 2].item(),
                    reward=episode_reward,
                    avg_reward=avg_reward,
                    policy_loss=policy_loss,
                    value_loss=value_loss,
                    entropy=entropy_loss,
                )
            )

    # Modeli Kaydet
    model_path = os.path.join(REPO_ROOT, "RL_PPO", "formasyon_sec", "formasyon_secim_modeli.pth")
    torch.save(model.state_dict(), model_path)
    print(f"✅ Model kaydedildi: {model_path}")


if __name__ == "__main__":
    train_formasyon_secim()