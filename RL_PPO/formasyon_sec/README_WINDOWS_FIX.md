# Windows PyTorch DLL Hatası Çözümü

## Sorun
```
OSError: [WinError 1114] A dynamic link library (DLL) initialization routine failed.
Error loading "c10.dll" or one of its dependencies.
```

## Çözüm Adımları

### 1. CPU-Only PyTorch Kurulumu (Önerilen)

Eğer CUDA kullanmıyorsanız, CPU-only PyTorch kurun:

```bash
# Mevcut PyTorch'u kaldır
pip uninstall torch torchvision torchaudio

# CPU-only PyTorch kur (Windows için)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

### 2. Visual C++ Redistributables Kurulumu

PyTorch için gerekli Visual C++ Redistributables'ı kurun:

1. [Microsoft Visual C++ Redistributables](https://aka.ms/vs/17/release/vc_redist.x64.exe) indirin
2. Kurulumu tamamlayın
3. Bilgisayarı yeniden başlatın

### 3. PyTorch Sürümünü Kontrol Et

```bash
python -c "import torch; print(torch.__version__)"
```

Eğer hata veriyorsa, PyTorch düzgün kurulmamış demektir.

### 4. Conda Environment Temizleme

```bash
# Mevcut environment'ı deaktif et
conda deactivate

# Yeni bir environment oluştur
conda create -n Sualti_new python=3.9
conda activate Sualti_new

# PyTorch CPU-only kur
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Diğer bağımlılıkları kur
pip install numpy ursina panda3d
```

### 5. Antivirus/Güvenlik Yazılımı Kontrolü

Bazı antivirus yazılımları DLL dosyalarını engelleyebilir:
- Windows Defender veya diğer antivirus yazılımlarında PyTorch klasörünü istisna listesine ekleyin
- `C:\ProgramData\Anaconda3\envs\Sualti\lib\site-packages\torch\` klasörünü istisna listesine ekleyin

### 6. Geçici Çözüm: Torch Import'unu Geciktir

Eğer yukarıdaki çözümler işe yaramazsa, `formasyon_sec_train.py` dosyasında torch import'unu geciktirebilirsiniz:

```python
# Torch import'unu fonksiyon içine taşı
def train_formasyon_secim():
    import torch
    import torch.nn as nn
    import torch.optim as optim
    # ... geri kalan kod
```

### 7. CUDA Sürümü Kontrolü (CUDA kullanıyorsanız)

Eğer CUDA kullanmak istiyorsanız:

```bash
# CUDA sürümünü kontrol et
nvidia-smi

# PyTorch CUDA sürümünü kontrol et
python -c "import torch; print(torch.version.cuda)"

# Uyumlu PyTorch CUDA sürümünü kur
# Örnek: CUDA 11.8 için
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

## Hızlı Test

PyTorch'un düzgün çalışıp çalışmadığını test edin:

```python
python -c "import torch; print('PyTorch:', torch.__version__); print('CUDA Available:', torch.cuda.is_available())"
```

Eğer bu komut hata vermeden çalışıyorsa, PyTorch düzgün kurulmuş demektir.

## Notlar

- CPU-only PyTorch, CUDA'dan daha yavaş olabilir ama daha stabil çalışır
- Eğitim süresi uzayabilir ama çalışır
- CUDA kullanmak istiyorsanız, CUDA ve PyTorch sürümlerinin uyumlu olduğundan emin olun
