# 🎨 3D Model Kullanım Kılavuzu

## 📋 Genel Bakış

ROV'lar için gerçek 3D modeller kullanabilirsiniz. Modeller `Models-3D/` klasöründen yüklenir.

---

## 📁 Klasör Yapısı

```
FiratRovNet-org/
├── Models-3D/
│   ├── README.md
│   ├── rov_model.obj          # Örnek OBJ modeli
│   ├── rov_model.glb          # Örnek GLB modeli
│   └── texture.png            # Texture dosyası (varsa)
├── FiratROVNet/
│   └── simulasyon.py          # ROV sınıfı (3D model desteği ile)
└── main.py                    # Ana dosya
```

---

## 🚀 Kullanım

### **1. Model Dosyasını Ekle**

3D model dosyanızı `Models-3D/` klasörüne kopyalayın:

```bash
cp /path/to/your/rov_model.obj Models-3D/
```

### **2. main.py'de Kullan**

```python
from FiratROVNet.simulasyon import Ortam

app = Ortam()
app.sim_olustur(
    n_rovs=4, 
    n_engels=25, 
    hedef_nokta=hedef_nokta,
    rov_model_yolu="rov_model.obj"  # Models-3D klasöründen yüklenecek
)
```

### **3. Çalıştır**

```bash
python main.py
```

---

## 📝 Desteklenen Formatlar

Ursina aşağıdaki formatları destekler:

| Format | Uzantı | Önerilen |
|--------|--------|----------|
| **OBJ** | `.obj` | ✅ En yaygın |
| **GLB/GLTF** | `.glb`, `.gltf` | ✅ Modern, optimize |
| **FBX** | `.fbx` | ⚠️ Autodesk format |
| **DAE** | `.dae` | ⚠️ Collada format |
| **BLEND** | `.blend` | ⚠️ Blender format (Blender gerekli) |

**Önerilen:** OBJ veya GLB formatı

---

## 🎯 Model Gereksinimleri

### **Ölçek:**
- Model 1 birim = 1 metre olmalı
- ROV boyutu yaklaşık 1-2 metre olmalı

### **Orientasyon:**
- **+Y:** Yukarı
- **+Z:** İleri
- **+X:** Sağ

### **Boyut:**
- Polygon sayısı: 1000-5000 (performans için)
- Texture: Aynı klasörde olmalı

### **Ölçeklendirme:**
Ursina'da `scale` parametresi ile ölçeklendirme yapılabilir:

```python
# Varsayılan ölçek
rov.scale = (1.5, 0.8, 2.5)  # (genişlik, yükseklik, uzunluk)
```

---

## 🔧 Örnekler

### **Örnek 1: Tek Model Tüm ROV'lar İçin**

```python
app.sim_olustur(
    n_rovs=4,
    n_engels=25,
    hedef_nokta=Vec3(40, 0, 60),
    rov_model_yolu="rov_model.obj"
)
```

### **Örnek 2: Model Yoksa Varsayılan Kullan**

```python
# Model yolu verilmezse varsayılan 'cube' kullanılır
app.sim_olustur(
    n_rovs=4,
    n_engels=25,
    hedef_nokta=Vec3(40, 0, 60)
    # rov_model_yolu belirtilmedi → cube kullanılır
)
```

### **Örnek 3: Sonradan Model Değiştir**

```python
# Simülasyon başladıktan sonra
rovs[0].model = "Models-3D/lider_rov.obj"
rovs[1].model = "Models-3D/takipci_rov.obj"
```

### **Örnek 4: Mutlak Yol Kullan**

```python
app.sim_olustur(
    n_rovs=4,
    rov_model_yolu="/absolute/path/to/model.obj"
)
```

---

## 🎨 Model Hazırlama

### **Blender'da Hazırlama:**

1. **Model Oluştur:**
   - ROV modelini 1x1x1 birim boyutunda oluşturun
   - Origin'i merkeze alın
   - +Y yukarı, +Z ileri olacak şekilde yönlendirin

2. **Export:**
   - File → Export → Wavefront (.obj)
   - Veya: File → Export → glTF 2.0 (.glb)

3. **Texture:**
   - Texture dosyalarını da `Models-3D/` klasörüne kopyalayın
   - OBJ dosyasında texture yolu göreceli olmalı

### **Model Kontrolü:**

```python
# Model yüklendi mi kontrol et
print(rovs[0].model)  # Model yolu veya 'cube'
```

---

## 📦 Model Kaynakları

Ücretsiz ROV modelleri için:

- **Sketchfab:** https://sketchfab.com
  - Arama: "ROV", "underwater robot", "AUV"
- **TurboSquid:** https://www.turbosquid.com
  - Ücretsiz modeller mevcut
- **Free3D:** https://free3d.com
  - Ücretsiz 3D modeller
- **BlenderKit:** Blender eklentisi
  - Blender içinden direkt indirme

---

## ⚠️ Sorun Giderme

### **Model Bulunamıyor:**

```
⚠️ [ROV] Model bulunamadı: rov_model.obj, varsayılan 'cube' kullanılıyor
```

**Çözüm:**
1. Model dosyasının `Models-3D/` klasöründe olduğundan emin olun
2. Dosya adını kontrol edin (büyük/küçük harf duyarlı)
3. Dosya yolunu kontrol edin

### **Model Yüklenmiyor:**

**Kontrol:**
```python
# Model yolu kontrolü
import os
model_path = "Models-3D/rov_model.obj"
print(f"Model var mı: {os.path.exists(model_path)}")
```

### **Model Görünmüyor:**

**Kontrol:**
- Model ölçeği çok küçük olabilir → `scale` değerini artırın
- Model pozisyonu yanlış olabilir → `position` kontrol edin
- Model rengi su ile aynı olabilir → `color` değiştirin

---

## 🔍 Model Yükleme Mantığı

```python
if model_yolu:
    # 1. Önce verilen yolu dene
    if os.path.exists(model_yolu):
        self.model = model_yolu
    else:
        # 2. Models-3D klasöründen dene
        models_dir = os.path.join(..., 'Models-3D')
        full_path = os.path.join(models_dir, model_yolu)
        if os.path.exists(full_path):
            self.model = full_path
        else:
            # 3. Bulunamazsa varsayılan cube kullan
            self.model = 'cube'
else:
    # Model yolu verilmediyse varsayılan cube
    self.model = 'cube'
```

---

## 📝 Özet

| Özellik | Açıklama |
|---------|----------|
| **Klasör** | `Models-3D/` |
| **Format** | OBJ, GLB, FBX, DAE, BLEND |
| **Kullanım** | `rov_model_yolu="model.obj"` |
| **Varsayılan** | `cube` (model yoksa) |
| **Ölçek** | `scale=(1.5, 0.8, 2.5)` |

**Sonuç:** Gerçek 3D modeller kullanarak simülasyonu daha gerçekçi hale getirebilirsiniz! 🎨

