# 🎨 3D Model Klasörü

Bu klasör ROV'lar için 3D modelleri içerir.

## 📁 Dosya Yapısı

```
Models-3D/
├── README.md (bu dosya)
├── rov_model.obj          # Örnek OBJ modeli
├── rov_model.glb          # Örnek GLB modeli
└── rov_model.fbx          # Örnek FBX modeli
```

## 🎯 Desteklenen Formatlar

Ursina aşağıdaki 3D model formatlarını destekler:

- **OBJ** (`.obj`) - En yaygın format
- **GLB/GLTF** (`.glb`, `.gltf`) - Modern, optimize format
- **FBX** (`.fbx`) - Autodesk format
- **DAE** (`.dae`) - Collada format
- **BLEND** (`.blend`) - Blender format (Blender yüklüyse)

## 📝 Kullanım

### 1. Model Dosyasını Ekle

3D model dosyanızı bu klasöre kopyalayın:

```bash
cp /path/to/your/rov_model.obj Models-3D/
```

### 2. main.py'de Kullan

```python
# Tek bir model tüm ROV'lar için
app.sim_olustur(
    n_rovs=4, 
    n_engels=25, 
    hedef_nokta=hedef_nokta,
    rov_model_yolu="rov_model.obj"  # Models-3D klasöründen yüklenecek
)
```

### 3. ROV Başına Farklı Model

Eğer her ROV için farklı model istiyorsanız, `sim_olustur()` fonksiyonunu güncelleyebilirsiniz:

```python
# Örnek: ROV-0 için özel model
rov_models = {
    0: "lider_rov.obj",
    1: "takipci_rov.obj",
    2: "takipci_rov.obj",
    3: "takipci_rov.obj"
}
```

## 🔧 Model Gereksinimleri

### Önerilen Özellikler:

1. **Ölçek:** Model 1 birim = 1 metre olmalı
2. **Orientasyon:** Model +Y yukarı, +Z ileri olmalı
3. **Boyut:** ROV boyutu yaklaşık 1-2 metre olmalı
4. **Polygon Sayısı:** Performans için 1000-5000 polygon önerilir
5. **Texture:** Texture dosyaları da aynı klasörde olmalı

### Model Hazırlama İpuçları:

1. **Blender'da Hazırlama:**
   - Modeli 1x1x1 birim boyutunda oluşturun
   - Origin'i merkeze alın
   - +Y yukarı, +Z ileri olacak şekilde yönlendirin
   - OBJ veya GLB olarak export edin

2. **Ölçeklendirme:**
   - Ursina'da `scale` parametresi ile ölçeklendirme yapılabilir
   - Varsayılan: `(1.5, 0.8, 2.5)`

## 📦 Örnek Model Kaynakları

Ücretsiz ROV modelleri için:

- **Sketchfab:** https://sketchfab.com (ROV, underwater robot araması)
- **TurboSquid:** https://www.turbosquid.com (ücretsiz modeller)
- **Free3D:** https://free3d.com
- **BlenderKit:** Blender eklentisi ile

## ⚠️ Notlar

- Model dosyası `Models-3D/` klasöründe olmalı
- Texture dosyaları da aynı klasörde olmalı
- Model bulunamazsa varsayılan `cube` modeli kullanılır
- Model yolu göreceli veya mutlak olabilir

## 🎨 Model Özelleştirme

ROV modelini değiştirmek için:

```python
# Simülasyon oluştururken
app.sim_olustur(n_rovs=4, rov_model_yolu="my_rov.obj")

# Veya sonradan değiştirmek için
rovs[0].model = "Models-3D/lider_rov.obj"
```



























