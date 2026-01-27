import torch
import numpy as np
from FiratROVNet.gnc import Filo
from FiratROVNet import senaryo
from lider_sec_model import LiderSecimAgi  # Model mimarisinin olduğu dosya


def test_model():
    # 1. Filo ve Model Kurulumu
    filo = Filo()
    model = LiderSecimAgi(input_dim=35, num_rovs=8)

    # Model ağırlıklarını yükle
    model_yolu = "lider_secim_modeli.pth"
    try:
        model.load_state_dict(torch.load(model_yolu))
        model.eval()  # Test modu (deaktif dropout/batchnorm)
        print(f"✅ Model yüklendi: {model_yolu}")
    except FileNotFoundError:
        print(f"❌ Hata: {model_yolu} bulunamadı! Önce eğitimi tamamlayın.")
        return

    print("\n--- Lider Seçim Yapay Zeka Testi Başlıyor ---\n")

    # 2. Test Döngüsü (5 farklı rastgele senaryo üzerinde dene)
    for i in range(5):
        # Filo üzerinden RL formatında veri üret (headless senaryo)
        # Bu fonksiyon hem girdiyi (state) hem de matematiksel doğruyu (target) döner
        data = filo.lider_sec_veri_uret()

        if data is None:
            continue

        # Girdiyi Tensor formatına getir
        state_tensor = torch.FloatTensor(data["input_state"]).unsqueeze(0)

        # 3. Model Tahmini (Inference)
        with torch.no_grad():
            id_logits, score_pred = model(state_tensor)

            # Sınıflandırma sonucunu al (En yüksek olasılıklı ID)
            tahmin_id = torch.argmax(id_logits, dim=1).item()
            tahmin_skor = score_pred.item()

        # 4. Sonuçları Karşılaştır
        gercek_id = data["target_lider_id"]
        gercek_skor = data["target_skor"]
        n_rovs = data["n_rovs"]

        print(f"Deney {i+1} ({n_rovs} ROV aktif):")
        print(f"  🎯 Matematiksel Lider ID : {gercek_id} (Skor: {gercek_skor:.6f})")
        print(f"  🤖 Yapay Zeka Tahmin ID  : {tahmin_id} (Skor: {tahmin_skor:.6f})")

        # Başarı kontrolü
        if tahmin_id == gercek_id:
            print("  ✅ SONUÇ: DOĞRU TAHMİN")
        else:
            print("  ❌ SONUÇ: YANLIŞ TAHMİN")

        skor_hata = abs(tahmin_skor - gercek_skor)
        print(f"  📊 Skor Sapması: {skor_hata:.8f}")
        print("-" * 50)

    # Temizlik (Senaryo modülünü kapat)
    senaryo.temizle()


if __name__ == "__main__":
    test_model()
