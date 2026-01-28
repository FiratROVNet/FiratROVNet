import os
import sys

# Ursina/Panda3D pencere loglarını sessize al (importlardan önce)
try:
    from panda3d.core import loadPrcFileData
    loadPrcFileData("", "window-type none")
    loadPrcFileData("", "audio-library-name null")
    loadPrcFileData("", "notify-level error")
    loadPrcFileData("", "default-directnotify-level error")
    loadPrcFileData("", "notify-level-display error")
except Exception:
    pass

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import torch
import numpy as np
from FiratROVNet.gnc import Filo
from FiratROVNet import senaryo
from formasyon_sec_rl_model import FormasyonSecimAgi


def test_formasyon_model():
    # 1. Filo ve Model Kurulumu
    filo = Filo()
    model = FormasyonSecimAgi(input_dim=230, num_formations=20)

    # Model ağırlıklarını yükle
    model_yolu = os.path.join(REPO_ROOT, "RL_PPO", "formasyon_sec", "formasyon_secim_modeli.pth")
    try:
        model.load_state_dict(torch.load(model_yolu))
        model.eval()  # Test modu
        print(f"✅ Model yüklendi: {model_yolu}")
    except FileNotFoundError:
        print(f"❌ Hata: {model_yolu} bulunamadı! Önce eğitimi tamamlayın.")
        return

    print("\n--- Formasyon Seçim Yapay Zeka Testi Başlıyor ---\n")

    # 2. Test Döngüsü (5 farklı rastgele senaryo üzerinde dene)
    for i in range(5):
        # Filo üzerinden RL formatında veri üret
        data = filo.uret_rl_egitim_verisi()

        if data is None:
            continue

        # Lider bilgilerini sakla (çıkışta kullanılacak)
        lider_pozisyon = data["lider_pozisyon"]  # (3,) - x, y, z
        lider_yaw = data["lider_yaw"]             # scalar - radyan veya derece

        # Giriş vektörünü hazırla
        lider_pos = torch.FloatTensor(lider_pozisyon)  # (3,)
        lider_yaw_tensor = torch.FloatTensor([lider_yaw])      # (1,)
        rov_filo_gps = torch.FloatTensor(data["rov_filo_gps"].flatten())  # (24,)
        hull_merkez = torch.FloatTensor(data["hull_merkez"])     # (2,)
        hull_noktalar = torch.FloatTensor(data["hull_noktalar"].flatten()[:200])  # (200,)
        
        state = torch.cat([
            lider_pos,
            lider_yaw_tensor,
            rov_filo_gps,
            hull_merkez,
            hull_noktalar
        ]).unsqueeze(0)

        # 3. Model Tahmini (Inference)
        with torch.no_grad():
            formation_id_logits, spacing_pred, yaw_pred, leader_pos_pred = model(state)

            # Sınıflandırma sonucunu al
            tahmin_formation_id = torch.argmax(formation_id_logits, dim=1).item()
            tahmin_spacing = spacing_pred.item()
            tahmin_yaw = yaw_pred.item()
            tahmin_leader_pos = leader_pos_pred[0].cpu().numpy()  # (3,) array

        # 4. Sonuçları Hazırla ve Döndür
        output_info = data["output"]
        
        if output_info is None:
            continue
        
        if isinstance(output_info, tuple) and len(output_info) >= 3:
            gercek_formation_id = int(output_info[0]) if output_info[0] is not None else 0
            gercek_spacing = float(output_info[1]) if output_info[1] is not None else 30.0
            gercek_yaw = float(output_info[2]) if output_info[2] is not None else 0.0
        else:
            continue

        # ÇIKIŞTAKİ SONUÇLAR - TAHMİNLER
        cikis = {
            "formasyon_id": tahmin_formation_id,
            "formasyon_araligi": tahmin_spacing,
            "lider_yaw": lider_yaw,  # Liderin yaw açısı
            "lider_konum": {
                "x": float(lider_pozisyon[0]),
                "y": float(lider_pozisyon[1]),
                "z": float(lider_pozisyon[2])
            }
        }

        print(f"Deney {i+1}:")
        print(f"  📋 Formasyon ID:")
        print(f"     🤖 Yapay Zeka Tahmin: {tahmin_formation_id}")
        print(f"     🎯 Matematiksel Gerçek: {gercek_formation_id}")
        
        if tahmin_formation_id == gercek_formation_id:
            print("     ✅ DOĞRU TAHMİN")
        else:
            print("     ❌ YANLIŞ TAHMİN")
        
        print(f"  📏 Formasyon Aralığı (metre):")
        print(f"     🤖 Yapay Zeka Tahmin: {tahmin_spacing:.2f} m")
        print(f"     🎯 Matematiksel Gerçek: {gercek_spacing:.2f} m")
        spacing_hata = abs(tahmin_spacing - gercek_spacing)
        print(f"     📊 Hata: {spacing_hata:.2f} m")
        
        print(f"  🔄 Lider Yaw Açısı:")
        print(f"     📍 Değer: {lider_yaw:.4f}")
        
        print(f"  📍 Lider Konumu (x, y, z):")
        print(f"     X: {lider_pozisyon[0]:.2f} m")
        print(f"     Y: {lider_pozisyon[1]:.2f} m")
        print(f"     Z: {lider_pozisyon[2]:.2f} m")
        
        print(f"\n  ✅ ÇIKIŞTAKİ SONUÇLAR:")
        print(f"     {cikis}")
        print("-" * 60)

    # Temizlik
    senaryo.temizle()


if __name__ == "__main__":
    test_formasyon_model()
