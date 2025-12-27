# 🚢 FiratROVNet Kullanım Kılavuzu

FiratROVNet, çoklu ROV (Remotely Operated Vehicle) simülasyonu ve yönetimi için geliştirilmiş bir Python kütüphanesidir.

---

## 📦 Kurulum

### Pip ile Kurulum

```bash
pip install git+https://github.com/FiratROVNet/FiratROVNet.git
```

### Gereksinimler

```bash
pip install -r requirements.txt
```

**Ana Bağımlılıklar:**
- `torch>=2.0.0` - Derin öğrenme
- `torch-geometric>=2.3.0` - Graf sinir ağları
- `ursina>=5.0.0` - 3D simülasyon motoru
- `numpy>=1.21.0` - Matematik işlemleri
- `scipy>=1.9.0` - Convex Hull hesaplamaları
- `matplotlib>=3.5.0` - Görselleştirme

---

## 🚀 Hızlı Başlangıç

### Basit Simülasyon

```python
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.iletisim import AkustikModem

# Simülasyon ortamı oluştur
app = Ortam()
app.sim_olustur(n_rovs=4, n_engels=15)

# Filo oluştur ve otomatik kurulum yap
filo = Filo()
tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,
    lider_id=0
)

# Simülasyonu çalıştır
app.run(interaktif=True)
```

---

## 📋 Filo Sınıfı - Ana API

### Temel Kullanım

```python
from FiratROVNet.gnc import Filo

filo = Filo()
```

### 1. Otomatik Kurulum

Tüm ROV'ları otomatik olarak kurar (modem, GNC, sensör, hedefler):

```python
tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,                    # ROV listesi
    lider_id=0,                       # Lider ROV ID
    modem_ayarlari={                  # Modem parametreleri (opsiyonel)
        'lider': {'gurultu_orani': 0.05, 'kayip_orani': 0.05},
        'takipci': {'gurultu_orani': 0.1, 'kayip_orani': 0.1}
    },
    baslangic_hedefleri={             # Başlangıç hedefleri (opsiyonel)
        0: (40, 0, 60),              # ROV-0: (x, y, z)
        1: (35, -10, 50)              # ROV-1: (x, y, z)
    },
    sensor_ayarlari={                 # Sensör ayarları (opsiyonel)
        'engel_mesafesi': 30.0,
        'iletisim_menzili': 40.0
    }
)
```

### 2. Hedef Atama - `git()`

ROV'a hedef koordinatı atar:

```python
# ROV-0'ı (40, 60, 0) koordinatına gönder
filo.git(0, 40, 60, 0)

# ROV-1'i (35, 50, -10) koordinatına gönder, AI kapalı
filo.git(1, 35, 50, -10, ai=False)

# Sadece x ve z koordinatları (y=derinlik varsayılan)
filo.git(2, 30, 40)
```

**Parametreler:**
- `rov_id`: ROV ID (0, 1, 2, ...)
- `x, z, y`: Hedef koordinatları (y=derinlik, opsiyonel)
- `ai`: AI aktif/pasif (varsayılan: True)

**Koordinat Sistemi:**
- `x`: Sağ-Sol (horizontal)
- `y`: İleri-Geri (forward-backward)
- `z`: Derinlik (depth, negatif = su altı)

### 3. Toplu Güncelleme - `guncelle_hepsi()`

Tüm ROV'ları GAT tahminleriyle günceller:

```python
# Her frame'de çağrılmalı
tahminler = [0, 1, 0, 2]  # Her ROV için GAT kodu
filo.guncelle_hepsi(tahminler)
```

**GAT Kodları:**
- `0`: OK (Normal durum)
- `1`: ENGEL (Engel tespit edildi)
- `2`: CARPISMA (Çarpışma riski)
- `3`: KOPUK (Bağlantı koptu)
- `5`: UZAK (Liderden uzak)

### 4. Manuel Hareket - `move()`

ROV'a güç bazlı hareket komutu verir:

```python
# ROV-0 %100 güçle ileri
filo.move(0, 'ileri', 1.0)

# ROV-1 %50 güçle sağa
filo.move(1, 'sag', 0.5)

# ROV-2 %30 güçle yukarı
filo.move(2, 'cik', 0.3)

# ROV-3 dur
filo.move(3, 'dur', 0.0)
```

**Yönler:**
- `'ileri'`, `'geri'`, `'sag'`, `'sol'`, `'cik'`, `'bat'`, `'dur'`

**Güç Parametresi:**
- `1.0` = %100 güç (maksimum hız)
- `0.5` = %50 güç (yarı hız)
- `0.0` = %0 güç (dur)

### 5. ROV Ayarları - `set()`

ROV ayarlarını değiştirir:

```python
# ROV-0'ı lider yap
filo.set(0, 'rol', 1)

# ROV-1'i kırmızı yap
filo.set(1, 'renk', (255, 0, 0))

# Sensör ayarı
filo.set(2, 'engel_mesafesi', 30.0)
```

**Desteklenen Ayarlar:**
- `'rol'`: Lider (1) veya Takipçi (0)
- `'renk'`: RGB tuple `(r, g, b)` veya renk ismi
- `'engel_mesafesi'`: Engel algılama menzili (metre)
- `'iletisim_menzili'`: İletişim menzili (metre)
- `'min_pil_uyarisi'`: Minimum pil seviyesi (0-100)

### 6. ROV Bilgisi - `get()`

ROV bilgilerini alır:

```python
# Pozisyon bilgisi
pozisyon = filo.get(0, 'gps')  # (x, y, z)

# Rol bilgisi
rol = filo.get(1, 'rol')  # 1 = Lider, 0 = Takipçi

# Sensör bilgileri
sensörler = filo.get(2, 'sensör')  # Dict

# Batarya seviyesi
batarya = filo.get(0, 'batarya')  # 0-100
```

**Desteklenen Veri Tipleri:**
- `'gps'`: Pozisyon (x, y, z)
- `'hiz'`: Hız vektörü
- `'batarya'`: Batarya seviyesi (0-100)
- `'rol'`: Rol (1=Lider, 0=Takipçi)
- `'renk'`: Renk bilgisi
- `'sensör'`: Tüm sensör ayarları
- `'sonar'`: Sonar mesafesi

### 7. Formasyon Sistemi - `formasyon()`

ROV'ları belirtilen formasyona sokar:

```python
# V şekli formasyon, 20 birim aralık
filo.formasyon("V_SHAPE", aralik=20)

# Elmas formasyonu, 3D mod
filo.formasyon("DIAMOND", aralik=25, is_3d=True)

# Sadece pozisyonları hesapla (ROV'ları hareket ettirme)
pozisyonlar = filo.formasyon("LINE", aralik=15, lider_koordinat=(10, 20, -5))
```

**Formasyon Tipleri:**
- `"LINE"`: Çizgi formasyonu
- `"V_SHAPE"`: V şekli
- `"DIAMOND"`: Elmas
- `"SQUARE"`: Kare
- `"CIRCLE"`: Daire
- `"ARROW"`: Ok
- `"WEDGE"`: Kama
- `"ECHELON"`: Eşelon
- `"COLUMN"`: Sütun
- `"SPREAD"`: Yayılım
- `"TRIANGLE"`: Üçgen
- `"CROSS"`: Haç
- `"STAGGERED"`: Kademeli
- `"WALL"`: Duvar
- `"STAR"`: Yıldız
- `"PHALANX"`: Falanks (sıkı düzen, askeri formasyon)
- `"RECTANGLE"`: Dikdörtgen formasyonu
- `"HEXAGON"`: Altıgen formasyonu
- `"WAVE"`: Dalga formasyonu
- `"SPIRAL"`: Spiral formasyonu

**Formasyon Seçimi - `formasyon_sec()`:**

Otomatik olarak en uygun formasyonu seçer:

```python
# Güvenlik hull'u kullanarak en uygun formasyonu seç
formasyon_id = filo.formasyon_sec(margin=30, is_3d=False, offset=20.0)

if formasyon_id is not None:
    print(f"Formasyon seçildi: {Formasyon.TIPLER[formasyon_id]}")
```

**Özellikler:**
- Lider GPS koordinatını öncelikli olarak kullanır
- Yaw açılarını dinamik olarak dener (0°, 90°, 180°, 270°)
- Hull merkezi fallback olarak kullanılır
- Formasyon bulunduğunda liderin yaw açısı otomatik set edilir

### 8. Hedef Belirleme - `hedef()`

Sadece lider ROV'un hedefini ayarlar:

```python
# Lider hedefi (50, 60, 0)
filo.hedef(50, 60)

# Mevcut hedefi al
mevcut_hedef = filo.hedef()  # (x, y, 0) veya None
```

---

## 📊 Senaryo Modülü

GUI olmadan (headless) simülasyon ortamları oluşturur:

```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)

# Veri al
batarya = senaryo.get(0, "batarya")
gps = senaryo.get(0, "gps")
sonar = senaryo.get(0, "sonar")

# Filo üzerinden erişim
if senaryo.filo:
    filo.git(0, 40, 60, 0)

# Temizle
senaryo.temizle()
```

---

## 🤖 GAT (Graf Dikkat Ağı) Sistemi

Yapay zeka tabanlı durum analizi:

```python
from FiratROVNet.gat import FiratAnalizci

# Model yükle
beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")

# Veri analiz et
veri = app.get_gat_data()
tahminler, _, _ = beyin.analiz_et(veri)

# Tahminleri kullan
filo.guncelle_hepsi(tahminler)
```

---

## 🎮 Konsol Erişimi

Simülasyon çalışırken konsola erişim:

```python
# main.py'de konsola ekle
app.konsola_ekle("filo", filo)
app.konsola_ekle("gnc", filo.sistemler)

# Konsolda kullanım
>>> filo.git(0, 50, 70, 0)
>>> filo.get(0, 'gps')
>>> filo.sistemler[0].hedef
```

---

## 💡 Örnek Kullanım Senaryoları

### Senaryo 1: Basit Kurulum

```python
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo

app = Ortam()
app.sim_olustur(n_rovs=4, n_engels=15)

filo = Filo()
filo.otomatik_kurulum(rovs=app.rovs)

app.run(interaktif=True)
```

### Senaryo 2: Formasyon ile Hareket

```python
# Formasyon oluştur
filo.formasyon("V_SHAPE", aralik=20)

# Lider hedefi belirle
filo.hedef(50, 60)

# Takipçiler otomatik olarak formasyonda kalır
```

### Senaryo 3: AI ile Kontrol

```python
from FiratROVNet.gat import FiratAnalizci

beyin = FiratAnalizci(model_yolu="rov_modeli_multi.pth")

def update():
    veri = app.get_gat_data()
    tahminler, _, _ = beyin.analiz_et(veri)
    filo.guncelle_hepsi(tahminler)

app.set_update_function(update)
app.run(interaktif=True)
```

### Senaryo 4: Manuel Kontrol

```python
# Güç bazlı manuel kontrol
filo.move(0, 'ileri', 1.0)   # %100 güçle ileri
filo.move(1, 'sag', 0.5)      # %50 güçle sağa
filo.move(2, 'cik', 0.3)       # %30 güçle yukarı
filo.move(3, 'dur', 0.0)       # Dur
```

---

## 🔧 Özellikler

### Otomatik Sistemler

- **Havuz Sınır Kontrolü**: ROV'lar havuz dışına çıkamaz
- **Çarpışma Sistemi**: Gerçekçi çarpışma fiziği
- **Lider ROV Özellikleri**: Otomatik su yüzeyinde kalır, batırılamaz
- **Formasyon Yönetimi**: Liderin yaw açısına göre dinamik formasyon

### Sensör Sistemi

- **Engel Algılama**: Lidar tabanlı engel tespiti
- **Sonar**: Mesafe ölçümü
- **Batarya**: Enerji yönetimi
- **İletişim**: Akustik modem simülasyonu

---

## 🐛 Hata Çözümü

### ROV'lar Hareket Etmiyor

```python
# AI kontrolünü aç
filo.git(0, 40, 60, 0, ai=True)

# Manuel kontrol dene
filo.move(0, 'ileri', 1.0)
```

### Formasyon Çalışmıyor

```python
# Formasyon seçimini manuel yap
formasyon_id = filo.formasyon_sec(margin=30)

# Veya manuel formasyon
filo.formasyon("V_SHAPE", aralik=20)
```

### Konsol Erişimi Yok

```python
# main.py'de konsola ekle
app.konsola_ekle("filo", filo)
app.konsola_ekle("gnc", filo.sistemler)
```

---

## 📚 İlgili Dokümantasyon

- **Senaryo Modülü**: `KILAVUZ/SENARYO_KULLANIM.md`
- **Konsol Erişimi**: `KILAVUZ/KONSOL_ERISIM.md`

---

## 🙏 Katkıda Bulunanlar

FiratROVNet Development Team

---

## 📝 Notlar

- `otomatik_kurulum()` çağrıldığında tüm ayarlar otomatik yapılır
- `guncelle_hepsi()` her frame'de çağrılmalı (update döngüsünde)
- Lider ROV (`rol=1`) batırılamaz, otomatik su yüzeyinde kalır
- Formasyon sistemi liderin yaw açısına göre dinamik olarak döndürülür
- Tüm koordinatlar Sim formatında: (x: sağ-sol, y: ileri-geri, z: derinlik)

