import os
import sys

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
import torch
import torch.nn as nn
import torch.nn.functional as F


class LiderSecimAgi(nn.Module):
    def __init__(self, input_dim=35, num_rovs=8):
        super(LiderSecimAgi, self).__init__()
        # Ortak Katmanlar
        self.fc1 = nn.Linear(input_dim, 128)
        self.fc2 = nn.Linear(128, 64)

        # Kafa 1: Sınıflandırma (Hangi ROV lider?)
        self.id_head = nn.Linear(64, num_rovs)

        # Kafa 2: Regresyon (Tahmin edilen skor ne?)
        self.score_head = nn.Linear(64, 1)

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))

        id_logits = self.id_head(x)  # Sınıflandırma için logitler
        score_pred = self.score_head(x)  # Skor tahmini

        return id_logits, score_pred
