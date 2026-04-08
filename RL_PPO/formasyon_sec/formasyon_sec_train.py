import os
import sys
import json
import random
import platform
import math

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import numpy as np


MAX_ROV_SAYISI = 8
LOKAL_PENCERE_METRE = 100.0
SABIT_GRID_BOYUTU = 32


def _safe_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _safe_int(value, default=0):
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return int(default)


def _normalize_xy_to_window(x, y, center_x, center_y, window_size):
    half = window_size / 2.0
    min_x = center_x - half
    min_y = center_y - half
    nx = (x - min_x) / window_size
    ny = (y - min_y) / window_size
    in_window = 0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0
    return float(np.clip(nx, 0.0, 1.0)), float(np.clip(ny, 0.0, 1.0)), in_window


def _normalize_z_to_window(z, center_z, window_size):
    half = window_size / 2.0
    nz = (z - (center_z - half)) / window_size
    return float(np.clip(nz, 0.0, 1.0))


def _normalize_yaw_to_0_1(yaw_value):
    yaw = float(yaw_value)
    if abs(yaw) <= (2.0 * math.pi * 2.0):
        yaw_rad = yaw % (2.0 * math.pi)
        return yaw_rad / (2.0 * math.pi)
    yaw_deg = yaw % 360.0
    return yaw_deg / 360.0


def _grid_index(norm_value, grid_size):
    idx = int(norm_value * grid_size)
    return min(max(idx, 0), grid_size - 1)


def local_window_multi_channel_map(rov_local_features, lidar_points_xy, leader_pos_xyz, window_size=LOKAL_PENCERE_METRE, grid_size=SABIT_GRID_BOYUTU):
    grid = np.zeros((2, grid_size, grid_size), dtype=np.float32)

    for rov in rov_local_features:
        if len(rov) < 4:
            continue
        x_norm, y_norm, _, varlik = rov
        if varlik < 0.5:
            continue
        gx = _grid_index(float(x_norm), grid_size)
        gy = _grid_index(float(y_norm), grid_size)
        grid[0, gy, gx] = 1.0

    if lidar_points_xy is None:
        return grid

    center_x = float(leader_pos_xyz[0])
    center_y = float(leader_pos_xyz[1])
    for p in np.asarray(lidar_points_xy, dtype=np.float32):
        if p.shape[0] < 2:
            continue
        x_norm, y_norm, in_window = _normalize_xy_to_window(
            float(p[0]),
            float(p[1]),
            center_x,
            center_y,
            window_size,
        )
        if not in_window:
            continue
        gx = _grid_index(x_norm, grid_size)
        gy = _grid_index(y_norm, grid_size)
        grid[1, gy, gx] = 1.0

    return grid


def _extract_lidar_points(hull_samples):
    if not isinstance(hull_samples, list):
        return np.zeros((0, 2), dtype=np.float32)

    points = []
    for item in hull_samples:
        if isinstance(item, dict):
            x = _safe_float(item.get("x", None), default=np.nan)
            y = _safe_float(item.get("y", None), default=np.nan)
        elif isinstance(item, (list, tuple)) and len(item) >= 2:
            x = _safe_float(item[0], default=np.nan)
            y = _safe_float(item[1], default=np.nan)
        else:
            continue

        if np.isnan(x) or np.isnan(y):
            continue
        points.append([x, y])

    if not points:
        return np.zeros((0, 2), dtype=np.float32)

    return np.asarray(points, dtype=np.float32)


def _extract_rov_position_with_presence(rov):
    pos = rov.get("pozisyon", {}) if isinstance(rov.get("pozisyon", {}), dict) else {}
    x = pos.get("x", None)
    y = pos.get("y", None)
    z = pos.get("z", None)

    if x is None or y is None or z is None:
        return np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    return np.array([
        _safe_float(x, 0.0),
        _safe_float(y, 0.0),
        _safe_float(z, 0.0),
        1.0,
    ], dtype=np.float32)


def _load_hull_information_records(file_path):
    if not os.path.exists(file_path):
        return []

    with open(file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    records = []
    if isinstance(data, dict):
        for _, value in data.items():
            if isinstance(value, list):
                records.extend([item for item in value if isinstance(item, dict)])
            elif isinstance(value, dict):
                records.append(value)
    elif isinstance(data, list):
        records = [item for item in data if isinstance(item, dict)]

    return records


def _hull_record_to_model_data(record):
    grup_bilgisi = record.get("grup_bilgisi", {}) if isinstance(record.get("grup_bilgisi", {}), dict) else {}
    rovlar = grup_bilgisi.get("rovlar", []) if isinstance(grup_bilgisi.get("rovlar", []), list) else []
    rovlar_sorted = sorted(rovlar, key=lambda r: _safe_int(r.get("rov_id", 999999)))

    lider_rov_id = _safe_int(record.get("lider_rov_id", -1), -1)
    lider_rov = None
    for rov in rovlar_sorted:
        if _safe_int(rov.get("rov_id", -1), -1) == lider_rov_id:
            lider_rov = rov
            break
    if lider_rov is None:
        for rov in rovlar_sorted:
            if _safe_int(rov.get("rol", 0), 0) == 1:
                lider_rov = rov
                break

    if lider_rov is None and rovlar_sorted:
        lider_rov = rovlar_sorted[0]

    if lider_rov is None:
        lider_pos = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    else:
        lider_pos = _extract_rov_position_with_presence(lider_rov)

    lider_yaw = _safe_float(record.get("lider_yaw", 0.0), 0.0)

    rov_filo_gps = []
    for rov in rovlar_sorted[:MAX_ROV_SAYISI]:
        rov_filo_gps.append(_extract_rov_position_with_presence(rov))
    while len(rov_filo_gps) < MAX_ROV_SAYISI:
        rov_filo_gps.append(np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32))
    rov_filo_gps = np.asarray(rov_filo_gps, dtype=np.float32)

    lidar_points = _extract_lidar_points(record.get("hull_samples", []))

    formasyon_id = _safe_int(record.get("formasyon_id", 0), 0)
    formasyon_aralik = _safe_float(record.get("formasyon_aralik", 30.0), 30.0)
    formasyon_yaw = _safe_float(record.get("formasyon_yaw", 0.0), 0.0)

    group_id = _safe_int(record.get("grup_id", grup_bilgisi.get("group_id", 0)), 0)
    n_rovs = _safe_int(grup_bilgisi.get("rov_sayisi", len(rovlar_sorted)), len(rovlar_sorted))

    return {
        "output": {
            "f_id": formasyon_id,
            "aralik": formasyon_aralik,
            "yaw": formasyon_yaw,
        },
        "n_rovs": n_rovs,
        "group_id": group_id,
        "lider_pozisyon": lider_pos,
        "lider_yaw": lider_yaw,
        "rov_filo_gps": rov_filo_gps,
        "lidar_noktalar": lidar_points,
    }


def _build_state_and_map(data):
    lider_raw = data["lider_pozisyon"]  # [x, y, z, varlik]
    leader_x = float(lider_raw[0])
    leader_y = float(lider_raw[1])
    leader_z = float(lider_raw[2])
    leader_presence = float(lider_raw[3])

    yaw_norm = _normalize_yaw_to_0_1(data.get("lider_yaw", 0.0))
    n_rovs_norm = float(np.clip(float(data.get("n_rovs", 0)) / float(MAX_ROV_SAYISI), 0.0, 1.0))

    lider_local = np.array([
        0.5,
        0.5,
        _normalize_z_to_window(leader_z, leader_z, LOKAL_PENCERE_METRE),
        leader_presence,
    ], dtype=np.float32)

    rov_local = []
    for rov in np.asarray(data["rov_filo_gps"], dtype=np.float32):
        x, y, z, varlik = float(rov[0]), float(rov[1]), float(rov[2]), float(rov[3])
        if varlik < 0.5:
            rov_local.append([0.0, 0.0, 0.0, 0.0])
            continue

        x_norm, y_norm, in_window = _normalize_xy_to_window(
            x,
            y,
            leader_x,
            leader_y,
            LOKAL_PENCERE_METRE,
        )
        if not in_window:
            rov_local.append([0.0, 0.0, 0.0, 0.0])
            continue

        z_norm = _normalize_z_to_window(z, leader_z, LOKAL_PENCERE_METRE)
        rov_local.append([x_norm, y_norm, z_norm, 1.0])

    rov_local = np.asarray(rov_local, dtype=np.float32)

    state = np.concatenate([
        lider_local,
        np.array([yaw_norm, n_rovs_norm], dtype=np.float32),
        rov_local.flatten(),
    ], axis=0).astype(np.float32)

    map_np = local_window_multi_channel_map(
        rov_local_features=rov_local,
        lidar_points_xy=data.get("lidar_noktalar", np.zeros((0, 2), dtype=np.float32)),
        leader_pos_xyz=lider_raw[:3],
        window_size=LOKAL_PENCERE_METRE,
        grid_size=SABIT_GRID_BOYUTU,
    )
    return state, map_np


def train_formasyon_secim():
    if platform.system() == 'Windows':
        try:
            import torch
            import torch.nn as nn
            import torch.optim as optim
            import torch.nn.functional as F
        except OSError as e:
            if "WinError 1114" in str(e) or "DLL" in str(e):
                print("❌ PyTorch DLL hatası tespit edildi!")
                print("   Lütfen README_WINDOWS_FIX.md dosyasındaki çözümleri uygulayın.")
                print("   Hızlı çözüm: pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu")
                print("   Visual C++ Redistributables kurulumu gerekebilir.")
                return
            raise
    else:
        import torch
        import torch.nn as nn
        import torch.optim as optim
        import torch.nn.functional as F

    from formasyon_sec_rl_model import FormasyonSecimAgi

    model = FormasyonSecimAgi(
        input_dim=38,
        num_formations=21,
        map_grid_size=SABIT_GRID_BOYUTU,
        map_input_channels=2,
    )
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    dataset_path = os.path.join(REPO_ROOT, "hull_information.json")
    records = _load_hull_information_records(dataset_path)
    if not records:
        print(f"❌ Eğitim verisi bulunamadı: {dataset_path}")
        return

    criterion_formation_id = nn.CrossEntropyLoss()
    criterion_spacing = nn.MSELoss()
    criterion_yaw = nn.MSELoss()
    criterion_leader_pos = nn.MSELoss()

    episode_rewards = []
    avg_reward_window = 100
    policy_losses = []
    value_losses = []
    entropy_losses = []

    print("🚀 Formasyon Seçim Eğitimi Başlıyor...")

    for epoch in range(1000):
        raw_record = random.choice(records)
        data = _hull_record_to_model_data(raw_record)
        if data is None:
            if epoch % 50 == 0:
                print("⚠️ [Eğitim] Veri üretimi başarısız (data=None)")
            continue

        state_np, map_np = _build_state_and_map(data)
        state = torch.FloatTensor(state_np).unsqueeze(0)
        map_tensor = torch.FloatTensor(map_np).unsqueeze(0)

        output_info = data["output"]
        group_id = float(data.get("group_id", 0))
        if output_info is None:
            if epoch % 50 == 0:
                print("⚠️ [Eğitim] Çıkış bilgisi yok (output=None)")
            continue

        if isinstance(output_info, dict):
            target_formation_id = torch.LongTensor([int(output_info.get("f_id", 0))])
            target_spacing = torch.FloatTensor([[float(output_info.get("aralik", 30.0))]])
            target_yaw = torch.FloatTensor([[_normalize_yaw_to_0_1(float(output_info.get("yaw", 0.0)))]] )
            target_leader_pos = torch.FloatTensor([[0.5, 0.5, 0.5]])
        elif isinstance(output_info, tuple) and len(output_info) >= 3:
            target_formation_id = torch.LongTensor([int(output_info[0]) if output_info[0] is not None else 0])
            target_spacing = torch.FloatTensor([[float(output_info[1]) if output_info[1] is not None else 30.0]])
            target_yaw = torch.FloatTensor([[_normalize_yaw_to_0_1(float(output_info[2]) if output_info[2] is not None else 0.0)]])
            target_leader_pos = torch.FloatTensor([[0.5, 0.5, 0.5]])
        else:
            continue

        optimizer.zero_grad()
        group_id_tensor = torch.FloatTensor([[group_id]])
        formation_id_logits, spacing_pred, yaw_pred, leader_pos_pred = model(
            state,
            group_id=group_id_tensor,
            map_tensor=map_tensor,
        )

        loss_formation_id = criterion_formation_id(formation_id_logits, target_formation_id)
        loss_spacing = criterion_spacing(spacing_pred, target_spacing)
        loss_yaw = criterion_yaw(yaw_pred, target_yaw)
        loss_leader_pos = criterion_leader_pos(leader_pos_pred, target_leader_pos)

        total_loss = loss_formation_id + 0.5 * loss_spacing + 0.3 * loss_yaw + 0.4 * loss_leader_pos

        with torch.no_grad():
            policy_loss = loss_formation_id.item()
            value_loss = (0.5 * loss_spacing.item() + 0.3 * loss_yaw.item() + 0.4 * loss_leader_pos.item())
            probs = F.softmax(formation_id_logits, dim=1)
            log_probs = F.log_softmax(formation_id_logits, dim=1)
            entropy = -(probs * log_probs).sum(dim=1).mean()
            entropy_loss = entropy.item()

            pred_id = torch.argmax(formation_id_logits, dim=1)
            accuracy = (pred_id == target_formation_id).float().mean().item()
            episode_reward = accuracy * 100.0 - total_loss.item() * 5.0

            episode_rewards.append(episode_reward)
            policy_losses.append(policy_loss)
            value_losses.append(value_loss)
            entropy_losses.append(entropy_loss)

            avg_reward = sum(episode_rewards[-avg_reward_window:]) / min(len(episode_rewards), avg_reward_window)

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

    model_path = os.path.join(REPO_ROOT, "RL_PPO", "formasyon_sec", "formasyon_secim_modeli.pth")
    torch.save(model.state_dict(), model_path)
    print(f"✅ Model kaydedildi: {model_path}")


if __name__ == "__main__":
    train_formasyon_secim()