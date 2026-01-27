import torch
import torch.nn as nn
import torch.optim as optim
from FiratROVNet.gnc import Filo
from lider_sec_model import LiderSecimAgi


def train_lider_secim():
    # 1. Kurulum
    filo = Filo()
    model = LiderSecimAgi()
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # Kayıp Fonksiyonları
    criterion_id = nn.CrossEntropyLoss()  # Sınıflandırma için
    criterion_score = nn.MSELoss()  # Regresyon için (Skor)

    print("🚀 Lider Seçim Eğitimi Başlıyor...")

    for epoch in range(100):  # 10000 iterasyon
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

        # Accuracy (sınıflandırma)
        with torch.no_grad():
            pred_id = torch.argmax(id_logits, dim=1)
            accuracy = (pred_id == target_id).float().mean().item()

        # Backward Pass
        total_loss.backward()
        optimizer.step()

        if epoch % 50 == 0:
            print(
                "Epoch: {epoch} | Total: {loss:.6f} | Loss(id): {loss_id:.6f} | "
                "Loss(score): {loss_score:.6f} | Acc: {acc:.4f} | "
                "Pred Score: {pred:.6f} | Real: {real:.6f}".format(
                    epoch=epoch,
                    loss=total_loss.item(),
                    loss_id=loss_id.item(),
                    loss_score=loss_score.item(),
                    acc=accuracy,
                    pred=score_pred.item(),
                    real=data["target_skor"],
                )
            )

    # Modeli Kaydet
    torch.save(model.state_dict(), "lider_secim_modeli.pth")
    print("✅ Model kaydedildi: lider_secim_modeli.pth")


if __name__ == "__main__":
    train_lider_secim()
