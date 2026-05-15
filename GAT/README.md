# GAT Eğitim Modülü Kullanım Kılavuzu

## Hızlı Başlangıç

### 1. Doğrudan Çalıştırma (Varsayılan Parametreler)

```bash
python GAT/gat_train.py
```

Bu komut varsayılan parametrelerle eğitimi başlatır:
- `epochs=5000`
- `lr=0.001`
- `hidden_channels=16`
- `num_heads=4`
- `dropout=0.1`

### 2. Python'dan Import Ederek Kullanma

```python
from GAT.gat_train import train, GAT_Modeli

# Varsayılan parametrelerle eğitim
model, best_loss = train()

# Özel parametrelerle eğitim
model, best_loss = train(
    epochs=10000,
    lr=0.0005,
    hidden_channels=32,
    num_heads=8,
    dropout=0.2,
    weight_decay=1e-5
)
```

### 3. Parametre Açıklamaları

- **epochs** (int): Eğitim epoch sayısı (varsayılan: 5000)
- **lr** (float): Learning rate (varsayılan: 0.001)
- **hidden_channels** (int): Gizli katman boyutu (varsayılan: 16)
- **num_heads** (int): Attention head sayısı (varsayılan: 4)
- **dropout** (float): Dropout oranı (varsayılan: 0.1)
- **weight_decay** (float): L2 regularization katsayısı (varsayılan: 1e-4)

### 4. Örnek Kullanımlar

#### Hızlı Test (Az Epoch)
```python
from GAT.gat_train import train

# Hızlı test için 100 epoch
model, best_loss = train(epochs=100)
```

#### Uzun Eğitim (Daha Fazla Epoch)
```python
from GAT.gat_train import train

# Uzun eğitim için 20000 epoch
model, best_loss = train(epochs=20000, lr=0.0005)
```

#### Büyük Model (Daha Fazla Hidden Channels)
```python
from GAT.gat_train import train

# Daha büyük model
model, best_loss = train(
    epochs=5000,
    hidden_channels=64,
    num_heads=8,
    dropout=0.15
)
```

#### Küçük Model (Hızlı Eğitim)
```python
from GAT.gat_train import train

# Daha küçük ve hızlı model
model, best_loss = train(
    epochs=3000,
    hidden_channels=8,
    num_heads=2,
    dropout=0.05
)
```

### 5. Model Dosyası

Eğitim sırasında en iyi model otomatik olarak kaydedilir:
- **Dosya adı**: `rov_modeli_multi.pth`
- **Konum**: `Models-AI/GAT/`
- **Kayıt koşulu**: Ortalama loss en iyi değerden düşükse ve epoch > 100

### 6. Çıktı Formatı

Eğitim sırasında şu bilgiler gösterilir:
- Her epoch'ta: Loss, Ortalama Loss, Doğruluk (Accuracy), Learning Rate
- Her 30 epoch'ta: Detaylı GAT kodları dağılımı (Gerçek ve Tahmin)
- Model kaydedildiğinde: Bildirim mesajı

### 7. Sistem Gereksinimleri

- PyTorch
- torch-geometric
- numpy
- FiratROVNet paketi (gnc modülü)

### 8. Sorun Giderme

#### Import Hatası
```python
# Eğer import hatası alırsanız, proje kök dizininden çalıştırdığınızdan emin olun
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
```

#### CUDA Hatası
Model otomatik olarak CPU'ya geçer. CUDA kullanmak için:
```python
# CUDA'nın yüklü olduğundan emin olun
import torch
print(torch.cuda.is_available())  # True olmalı
```

#### Senaryo Verisi Üretilemedi
Eğer "Senaryo verisi üretilemedi" hatası alırsanız:
- Senaryo modülünün doğru çalıştığından emin olun
- ROV sayısının 4, 6 veya 8 olduğundan emin olun
- Ada sayısının 2-5 arasında olduğundan emin olun
