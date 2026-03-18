import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import torch
import torch.nn as nn
import torch.nn.functional as F


class FormasyonSecimAgi(nn.Module):
    
    def __init__(
        self,
        input_dim=38,
        num_formations=21,
        map_grid_size=32,
        map_input_channels=2,
        map_feature_dim=64,
    ):
        super(FormasyonSecimAgi, self).__init__()
        self.base_input_dim = input_dim
        self.input_dim_with_group = input_dim + 1
        self.map_grid_size = int(map_grid_size)
        self.map_input_channels = int(map_input_channels)
        self.map_feature_dim = map_feature_dim
        self.cnn_out_channels = 16

        self.harita_cnn = nn.Sequential(
            nn.Conv2d(in_channels=self.map_input_channels, out_channels=16, kernel_size=3, padding=1),
            nn.ReLU(),
            nn.Conv2d(in_channels=16, out_channels=self.cnn_out_channels, kernel_size=3, padding=1),
            nn.ReLU(),
        )

        self.map_flat_dim = self.cnn_out_channels * self.map_grid_size * self.map_grid_size
        self.map_fc = nn.Linear(self.map_flat_dim, self.map_feature_dim)
        self.map_norm = nn.LayerNorm(self.map_feature_dim)
        
        # Ortak Katmanlar
        self.fc1 = nn.Linear(self.input_dim_with_group + self.map_feature_dim, 128)
        self.fc2 = nn.Linear(128, 64)

        # Başlık 1: Formasyon Tipi Sınıflandırması
        self.formation_id_head = nn.Linear(64, num_formations)

        # Başlık 2: Aralık Regresyonu (Spacing)
        self.spacing_head = nn.Linear(64, 1)
        
        # Başlık 3: Yaw Regresyonu (Lider'in yaw açısı)
        self.yaw_head = nn.Linear(64, 1)
        
        # Başlık 4: Lider Konum Regresyonu (X, Y, Z koordinatları)
        self.leader_position_head = nn.Linear(64, 3)
    
    def _hazirla_harita_ozellikleri(self, map_tensor, batch_size, device, dtype):
        if map_tensor is None:
            return torch.zeros((batch_size, self.map_feature_dim), device=device, dtype=dtype)

        if not torch.is_tensor(map_tensor):
            map_tensor = torch.tensor(map_tensor, device=device, dtype=dtype)
        else:
            map_tensor = map_tensor.to(device=device, dtype=dtype)

        if map_tensor.dim() == 2:
            map_tensor = map_tensor.unsqueeze(0).unsqueeze(0).repeat(1, self.map_input_channels, 1, 1)
        elif map_tensor.dim() == 3:
            if map_tensor.size(0) == self.map_input_channels:
                map_tensor = map_tensor.unsqueeze(0)
            else:
                map_tensor = map_tensor.unsqueeze(1).repeat(1, self.map_input_channels, 1, 1)
        elif map_tensor.dim() != 4:
            raise ValueError(
                f"map_tensor shape desteklenmiyor: {tuple(map_tensor.shape)} | beklenen: (H,W), (C,H,W) veya (B,C,H,W)"
            )

        if map_tensor.size(1) != self.map_input_channels:
            if map_tensor.size(1) == 1 and self.map_input_channels == 2:
                map_tensor = map_tensor.repeat(1, 2, 1, 1)
            else:
                raise ValueError(
                    f"map_tensor kanal sayısı hatalı: {map_tensor.size(1)} != {self.map_input_channels}"
                )

        if map_tensor.size(2) != self.map_grid_size or map_tensor.size(3) != self.map_grid_size:
            raise ValueError(
                f"map_tensor boyutu sabit olmalı: ({self.map_grid_size}, {self.map_grid_size}), gelen: ({map_tensor.size(2)}, {map_tensor.size(3)})"
            )

        if map_tensor.size(0) == 1 and batch_size > 1:
            map_tensor = map_tensor.expand(batch_size, -1, -1, -1)
        elif map_tensor.size(0) != batch_size:
            raise ValueError(
                f"map_tensor batch boyutu x ile uyumsuz: {map_tensor.size(0)} != {batch_size}"
            )

        map_tensor = torch.clamp(map_tensor, 0.0, 1.0)

        cnn_features = self.harita_cnn(map_tensor)
        pooled_flat = cnn_features.reshape(batch_size, -1)
        map_features = F.relu(self.map_fc(pooled_flat))
        map_features = self.map_norm(map_features)
        return map_features

    def forward(self, x, group_id=None, map_tensor=None):
        if x.dim() == 1:
            x = x.unsqueeze(0)

        if x.size(-1) == self.base_input_dim:
            if group_id is None:
                group_id = torch.zeros((x.size(0), 1), device=x.device, dtype=x.dtype)
            else:
                if not torch.is_tensor(group_id):
                    group_id = torch.tensor(group_id, device=x.device, dtype=x.dtype)
                else:
                    group_id = group_id.to(device=x.device, dtype=x.dtype)

                if group_id.dim() == 0:
                    group_id = group_id.view(1, 1).expand(x.size(0), 1)
                elif group_id.dim() == 1:
                    if group_id.size(0) == x.size(0):
                        group_id = group_id.unsqueeze(1)
                    elif group_id.size(0) == 1:
                        group_id = group_id.view(1, 1).expand(x.size(0), 1)
                    else:
                        raise ValueError(
                            f"group_id batch boyutu x ile uyumsuz: {group_id.size(0)} != {x.size(0)}"
                        )
                elif group_id.dim() == 2:
                    if group_id.size(1) != 1:
                        raise ValueError(
                            f"group_id şekli (batch, 1) olmalı, gelen: {tuple(group_id.shape)}"
                        )
                    if group_id.size(0) == 1 and x.size(0) > 1:
                        group_id = group_id.expand(x.size(0), 1)
                    elif group_id.size(0) != x.size(0):
                        raise ValueError(
                            f"group_id batch boyutu x ile uyumsuz: {group_id.size(0)} != {x.size(0)}"
                        )
                else:
                    raise ValueError(
                        f"group_id desteklenmeyen shape: {tuple(group_id.shape)}"
                    )

            x = torch.cat([x, group_id], dim=1)
        elif x.size(-1) != self.input_dim_with_group:
            raise ValueError(
                f"Beklenen giriş boyutu {self.base_input_dim} veya {self.input_dim_with_group}, gelen: {x.size(-1)}"
            )

        map_features = self._hazirla_harita_ozellikleri(
            map_tensor=map_tensor,
            batch_size=x.size(0),
            device=x.device,
            dtype=x.dtype,
        )

        x = torch.cat([x, map_features], dim=1)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        formation_id_logits = self.formation_id_head(x)      # (batch, 20)
        spacing_pred = self.spacing_head(x)                   # (batch, 1)
        yaw_pred = self.yaw_head(x)                          # (batch, 1)
        leader_position_pred = self.leader_position_head(x)  # (batch, 3) - X, Y, Z

        return formation_id_logits, spacing_pred, yaw_pred, leader_position_pred
