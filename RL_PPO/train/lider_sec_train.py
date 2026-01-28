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
from RL_PPO.lider_sec.lider_sec_model import LiderSecimAgi


def train_lider_secim():
    # 1. Kurulum
    filo = Filo()
    model = LiderSecimAgi()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # Kayıp Fonksiyonları
    criterion_id = nn.CrossEntropyLoss()  # Sınıflandırma için
    criterion_score = nn.MSELoss()  # Regresyon için (Skor)

    # RL Metrikleri için değişkenler
    episode_rewards = []  # Her episode için reward
    avg_reward_window = 100  # Son N episode için ortalama
    policy_losses = []  # Policy loss'ları
    value_losses = []  # Value loss'ları
    entropy_losses = []  # Entropy loss'ları

    print("🚀 Lider Seçim Eğitimi Başlıyor...")

    for epoch in range(1000):  # 10000 iterasyon
        # Veri Üret
        data = filo.lider_sec_veri_uret()
        if data is None:
            continue

        # Tensors
        state = torch.FloatTensor(data["state"]).unsqueeze(0)
        target_id = torch.LongTensor([data["target_id"]])
        target_skor = torch.FloatTensor([data["target_skor"]]).unsqueeze(0)

        # Forward Pass
        optimizer.zero_grad()
        id_logits, score_pred = model(state)

        # Loss Calculation
        loss_id = criterion_id(id_logits, target_id)
        loss_score = criterion_score(score_pred, target_skor)

        # Toplam Kayıp (Ağırlıklı birleştirilebilir)
        total_loss = loss_id + loss_score

        # RL Metrikleri Hesaplama
        with torch.no_grad():
            # Policy Loss (CrossEntropyLoss)
            policy_loss = loss_id.item()
            
            # Value Loss (MSELoss)
            value_loss = loss_score.item()
            
            # Entropy Loss (Logits'lerden entropy hesapla)
            probs = F.softmax(id_logits, dim=1)
            log_probs = F.log_softmax(id_logits, dim=1)
            entropy = -(probs * log_probs).sum(dim=1).mean()
            entropy_loss = entropy.item()
            
            # Reward Hesaplama (Accuracy ve skor hatasına göre)
            pred_id = torch.argmax(id_logits, dim=1)
            accuracy = (pred_id == target_id).float().mean().item()
            # Reward: Accuracy yüksek, loss düşük olmalı
            # Reward = accuracy * 100 - total_loss (normalize edilmiş)
            episode_reward = accuracy * 100.0 - total_loss.item() * 10.0
            
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
                "Loss(score): {loss_score:.6f} | Acc: {acc:.4f} | "
                "Pred Score: {pred:.6f} | Real: {real:.6f} | "
                "Reward: {reward:.2f} | Avg Reward: {avg_reward:.2f} | "
                "Policy Loss: {policy_loss:.6f} | Value Loss: {value_loss:.6f} | Entropy: {entropy:.6f}".format(
                    epoch=epoch,
                    loss=total_loss.item(),
                    loss_id=loss_id.item(),
                    loss_score=loss_score.item(),
                    acc=accuracy,
                    pred=score_pred.item(),
                    real=data["target_skor"],
                    reward=episode_reward,
                    avg_reward=avg_reward,
                    policy_loss=policy_loss,
                    value_loss=value_loss,
                    entropy=entropy_loss,
                )
            )

    # Modeli Kaydet
    model_path = os.path.join(REPO_ROOT, "RL_PPO", "lider_sec", "lider_secim_modeli.pth")
    torch.save(model.state_dict(), model_path)
    print(f"✅ Model kaydedildi: {model_path}")


if __name__ == "__main__":
    train_lider_secim()
