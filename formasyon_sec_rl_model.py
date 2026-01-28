import torch
import torch.nn as nn
import torch.nn.functional as F


class FormasyonSecimAgi(nn.Module):
    
    def __init__(self, input_dim=230, num_formations=20):
        super(FormasyonSecimAgi, self).__init__()
        
        # Ortak Katmanlar
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 64)

        # Başlık 1: Formasyon Tipi Sınıflandırması
        self.formation_id_head = nn.Linear(64, num_formations)

        # Başlık 2: Aralık Regresyonu (Spacing)
        self.spacing_head = nn.Linear(64, 1)
        
        # Başlık 3: Yaw Regresyonu (Lider'in yaw açısı)
        self.yaw_head = nn.Linear(64, 1)
        
        # Başlık 4: Lider Konum Regresyonu (X, Y, Z koordinatları)
        self.leader_position_head = nn.Linear(64, 3)
    
    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        formation_id_logits = self.formation_id_head(x)      # (batch, 20)
        spacing_pred = self.spacing_head(x)                   # (batch, 1)
        yaw_pred = self.yaw_head(x)                          # (batch, 1)
        leader_position_pred = self.leader_position_head(x)  # (batch, 3) - X, Y, Z

        return formation_id_logits, spacing_pred, yaw_pred, leader_position_pred
