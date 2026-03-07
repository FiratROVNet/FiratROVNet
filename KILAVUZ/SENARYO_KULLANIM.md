# 📊 Senaryo Modülü Kullanım Rehberi

`FiratROVNet.senaryo` modülü, GUI olmadan (headless) simülasyon ortamları oluşturur ve yapay zeka algoritmalarını eğitmek için veri üretir.

---

## 📋 İçindekiler

1. [Temel Kullanım](#temel-kullanım)
2. [Parametreler](#parametreler)
3. [Veri Erişimi](#veri-erişimi)
4. [Simülasyon Adımları](#simülasyon-adımları)
5. [Örnekler](#örnekler)

---

## 🚀 Temel Kullanım

### Basit Senaryo Oluşturma

```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20, havuz_genisligi=200)

# Veri al
batarya = senaryo.get(0, "batarya")
gps = senaryo.get(0, "gps")
sonar = senaryo.get(0, "sonar")

print(f"ROV-0 Batarya: {batarya}")
print(f"ROV-0 GPS: {gps}")
print(f"ROV-0 Sonar: {sonar}")

# Temizle
senaryo.temizle()
```

### Filo Üzerinden Erişim

```python
from FiratROVNet import senaryo

senaryo.uret(n_rovs=3, n_engels=15)

# Filo üzerinden veri al
if senaryo.filo:
    batarya = senaryo.filo.get(0, "batarya")
    gps = senaryo.filo.get(0, "gps")
    print(f"Batarya: {batarya}, GPS: {gps}")
```

---

## ⚙️ Parametreler

### `uret()` Fonksiyonu

```python
senaryo.uret(
    n_rovs=3,                    # ROV sayısı
    n_engels=15,                 # Engel sayısı
    havuz_genisligi=200,         # Havuz genişliği
    engel_tipleri=None,          # Engel tipleri listesi
    baslangic_pozisyonlari=None, # ROV başlangıç pozisyonları
    modem_ayarlari=None,         # Modem ayarları
    sensor_ayarlari=None         # Sensör ayarları
)
```

#### Engel Tipleri

```python
# Farklı engel tipleri
engel_tipleri = ['kaya'] * 10 + ['agac'] * 5

senaryo.uret(
    n_rovs=3,
    n_engels=15,
    engel_tipleri=engel_tipleri
)
```

#### Başlangıç Pozisyonları

```python
baslangic_pozisyonlari = {
    0: (0, -5, 0),      # ROV-0 merkez
    1: (10, -5, 10),    # ROV-1 sağ-ileri
    2: (-10, -5, -10),  # ROV-2 sol-geri
}

senaryo.uret(
    n_rovs=3,
    baslangic_pozisyonlari=baslangic_pozisyonlari
)
```

#### Sensör Ayarları

Sensör ayarları `config.py`'deki `SensorAyarlari` sınıfından alınır ve GAT limitleri ile tutarlıdır:

```python
from FiratROVNet.config import SensorAyarlari

# Varsayılan ayarları kullan (GAT limitleri ile uyumlu)
senaryo.uret(
    n_rovs=3,
    sensor_ayarlari=None  # Varsayılan: SensorAyarlari.LIDER ve TAKIPCI
)

# Özel sensör ayarları
sensor_ayarlari = {
    'lider': {
        'engel_mesafesi': 20.0,      # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 35.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.2,       # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 8.0       # GATLimitleri.CARPISMA ile aynı
    },
    'takipci': {
        'engel_mesafesi': 20.0,       # GATLimitleri.ENGEL ile aynı
        'iletisim_menzili': 35.0,     # GATLimitleri.KOPMA ile aynı
        'min_pil_uyarisi': 0.15,      # Normalize edilmiş (0.0-1.0)
        'kacinma_mesafesi': 8.0       # GATLimitleri.CARPISMA ile aynı
    }
}

senaryo.uret(
    n_rovs=3,
    sensor_ayarlari=sensor_ayarlari
)
```

**Önemli**: Sensör ayarları GAT limitleri ile uyumlu olmalıdır:
- `engel_mesafesi` >= `GATLimitleri.ENGEL` (20.0)
- `iletisim_menzili` >= `GATLimitleri.KOPMA` (35.0)
- `kacinma_mesafesi` <= `GATLimitleri.CARPISMA` (8.0)

#### Modem Ayarları

Modem ayarları `config.py`'deki `ModemAyarlari` sınıfından alınır:

```python
from FiratROVNet.config import ModemAyarlari

# Varsayılan modem ayarlarını kullan
senaryo.uret(
    n_rovs=3,
    modem_ayarlari=None  # Varsayılan: ModemAyarlari.LIDER ve TAKIPCI
)

# Özel modem ayarları
modem_ayarlari = {
    'lider': {
        'gurultu_orani': 0.05,    # Gürültü oranı (0.0-1.0)
        'kayip_orani': 0.1,       # Paket kayıp oranı (0.0-1.0)
        'gecikme': 0.5            # Gecikme (saniye)
    },
    'takipci': {
        'gurultu_orani': 0.1,     # Gürültü oranı (0.0-1.0)
        'kayip_orani': 0.1,       # Paket kayıp oranı (0.0-1.0)
        'gecikme': 0.5            # Gecikme (saniye)
    }
}

senaryo.uret(
    n_rovs=3,
    modem_ayarlari=modem_ayarlari
)
```

---

## 📊 Veri Erişimi

### `get()` Fonksiyonu

```python
# ROV verisine erişim
veri = senaryo.get(rov_id, veri_tipi)
```

#### Desteklenen Veri Tipleri

| Veri Tipi | Açıklama | Dönüş Değeri |
|-----------|----------|--------------|
| `"batarya"` | Batarya seviyesi | `float` (0-1) |
| `"gps"` | GPS koordinatları | `numpy.array([x, y, z])` |
| `"hiz"` | Hız vektörü | `numpy.array([vx, vy, vz])` |
| `"sonar"` | Sonar mesafesi | `float` |
| `"rol"` | ROV rolü | `int` (0=takipçi, 1=lider) |
| `"engel_mesafesi"` | Engel tespit mesafesi | `float` |
| `"iletisim_menzili"` | İletişim menzili | `float` |

#### Örnekler

```python
# Batarya seviyesi
batarya = senaryo.get(0, "batarya")
print(f"Batarya: {batarya:.2%}")  # % formatında

# GPS koordinatları
gps = senaryo.get(0, "gps")
print(f"GPS: X={gps[0]:.2f}, Y={gps[1]:.2f}, Z={gps[2]:.2f}")

# Hız vektörü
hiz = senaryo.get(0, "hiz")
print(f"Hız: {hiz}")

# Sonar mesafesi
sonar = senaryo.get(0, "sonar")
print(f"Sonar: {sonar}")
```

### Filo Üzerinden Erişim

```python
# Senaryo.filo üzerinden direkt erişim
if senaryo.filo:
    # Tüm Filo metodları kullanılabilir
    batarya = senaryo.filo.get(0, "batarya")
    senaryo.filo.git(0, 50, 60, -10)  # Hedef ata
    senaryo.filo.set(0, "engel_mesafesi", 25.0)  # Ayar değiştir
```

---

## 🔄 Simülasyon Adımları

### `guncelle()` Fonksiyonu

```python
# Senaryo ortamını bir adım güncelle
senaryo.guncelle(delta_time=0.016)  # 16ms (60 FPS)
```

### Örnek: Simülasyon Döngüsü

```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=2, n_engels=10)

# Hedef ata
senaryo.git(0, 50, 60, -10)

# Simülasyon döngüsü
for adim in range(100):
    senaryo.guncelle(0.016)  # 16ms
    
    # Her 10 adımda bir veri al
    if adim % 10 == 0:
        gps = senaryo.get(0, "gps")
        hiz = senaryo.get(0, "hiz")
        print(f"Adım {adim}: GPS={gps}, Hız={hiz}")

senaryo.temizle()
```

---

## 📝 Örnekler

### Örnek 1: Basit Veri Toplama

```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20)

# Tüm ROV'ların verilerini topla
for i in range(4):
    batarya = senaryo.get(i, "batarya")
    gps = senaryo.get(i, "gps")
    rol = senaryo.get(i, "rol")
    print(f"ROV-{i} (Rol: {rol}): Batarya={batarya:.2f}, GPS={gps}")

senaryo.temizle()
```

### Örnek 2: AI Eğitimi İçin Veri Seti

```python
from FiratROVNet import senaryo
import numpy as np

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20)

# Veri seti oluştur
veri_seti = []

for adim in range(1000):  # 1000 adım veri topla
    senaryo.guncelle(0.016)
    
    # Her ROV için veri topla
    adim_verisi = {}
    for rov_id in range(4):
        adim_verisi[rov_id] = {
            'gps': senaryo.get(rov_id, "gps"),
            'hiz': senaryo.get(rov_id, "hiz"),
            'batarya': senaryo.get(rov_id, "batarya"),
            'sonar': senaryo.get(rov_id, "sonar"),
            'rol': senaryo.get(rov_id, "rol")
        }
    
    veri_seti.append(adim_verisi)
    
    if adim % 100 == 0:
        print(f"Adım {adim}: {len(veri_seti)} veri noktası toplandı")

print(f"✅ Toplam {len(veri_seti)} adım veri toplandı")

senaryo.temizle()
```

### Örnek 3: Farklı Senaryolar

```python
from FiratROVNet import senaryo

# Senaryo 1: Küçük havuz, az engel
senaryo.uret(n_rovs=2, n_engels=5, havuz_genisligi=100)
# ... veri topla ...
senaryo.temizle()

# Senaryo 2: Büyük havuz, çok engel
senaryo.uret(n_rovs=6, n_engels=50, havuz_genisligi=500)
# ... veri topla ...
senaryo.temizle()

# Senaryo 3: Özel başlangıç pozisyonları
baslangic = {
    0: (0, -5, 0),
    1: (20, -5, 20),
    2: (-20, -5, -20)
}
senaryo.uret(n_rovs=3, baslangic_pozisyonlari=baslangic)
# ... veri topla ...
senaryo.temizle()
```

### Örnek 4: Filo Komutları

```python
from FiratROVNet import senaryo

senaryo.uret(n_rovs=3, n_engels=15)

# Filo üzerinden komutlar
if senaryo.filo:
    # Hedef ata
    senaryo.filo.git(0, 50, 60, -10)
    
    # Ayar değiştir
    senaryo.filo.set(0, "engel_mesafesi", 25.0)
    
    # Veri al
    batarya = senaryo.filo.get(0, "batarya")
    print(f"Batarya: {batarya}")

senaryo.temizle()
```

---

## 🔧 Diğer Fonksiyonlar

### `set()` - Ayar Değiştirme

```python
# ROV ayarını değiştir
senaryo.set(0, "engel_mesafesi", 20.0)      # GATLimitleri.ENGEL ile uyumlu
senaryo.set(0, "iletisim_menzili", 35.0)    # GATLimitleri.KOPMA ile uyumlu
senaryo.set(0, "kacinma_mesafesi", 8.0)     # GATLimitleri.CARPISMA ile uyumlu
senaryo.set(0, "min_pil_uyarisi", 0.2)      # Normalize edilmiş (0.0-1.0)
```

**Not**: Ayarları değiştirirken GAT limitleri ile uyumlu olmasına dikkat edin. `config.py`'deki `SensorAyarlari` sınıfını referans alabilirsiniz.

### `git()` - Hedef Atama

```python
# ROV'a hedef ata
senaryo.git(0, 50, 60, -10)  # (x, z, y)
```

### `temizle()` - Temizlik

```python
# Senaryo ortamını temizle
senaryo.temizle()
```

---

## ⚠️ Önemli Notlar

1. **Headless Mod**: Senaryo modülü GUI olmadan çalışır (headless)
2. **Temizlik**: Kullanımdan sonra `senaryo.temizle()` çağırın
3. **Filo Erişimi**: `senaryo.filo` sadece senaryo aktifken erişilebilir
4. **Performans**: Headless mod GUI'den çok daha hızlıdır
5. **Config Tutarlılığı**: Sensör ve modem ayarları `config.py`'deki `SensorAyarlari` ve `ModemAyarlari` sınıflarından alınır
6. **GAT Limitleri**: Sensör ayarları GAT limitleri (`GATLimitleri`) ile uyumlu olmalıdır:
   - `engel_mesafesi` >= 20.0 (GATLimitleri.ENGEL)
   - `iletisim_menzili` >= 35.0 (GATLimitleri.KOPMA)
   - `kacinma_mesafesi` <= 8.0 (GATLimitleri.CARPISMA)
7. **Varsayılan Değerler**: Eğer `sensor_ayarlari=None` veya `modem_ayarlari=None` ise, `config.py`'deki varsayılan değerler kullanılır

---

## 📚 İlgili Dokümantasyon

- [Filo Kullanım Rehberi](FILO_KULLANIM.md)
- [GAT Kodları Rehberi](GAT_KODLARI_RENKLER.md)
- [Simülasyon Modülü](../FiratROVNet/simulasyon.py)
- [Config Modülü](../FiratROVNet/config.py) - GAT limitleri ve sensör ayarları

---

## 🔧 Config Modülü Kullanımı

Senaryo modülü, `config.py`'deki ortak ayarları kullanır:

```python
from FiratROVNet.config import GATLimitleri, SensorAyarlari, ModemAyarlari

# GAT limitlerini görüntüle
print(f"Çarpışma limiti: {GATLimitleri.CARPISMA}")  # 8.0
print(f"Engel limiti: {GATLimitleri.ENGEL}")        # 20.0
print(f"Kopma limiti: {GATLimitleri.KOPMA}")        # 35.0
print(f"Uzak limiti: {GATLimitleri.UZAK}")          # 60.0

# Sensör ayarlarını kullan
lider_ayarlari = SensorAyarlari.LIDER.copy()
takipci_ayarlari = SensorAyarlari.TAKIPCI.copy()

# Senaryo oluştururken kullan
senaryo.uret(
    n_rovs=3,
    sensor_ayarlari={
        'lider': lider_ayarlari,
        'takipci': takipci_ayarlari
    }
)
```

Bu yaklaşım, eğitim ve kullanım arasında tutarlılık sağlar.

---

**Son Güncelleme**: 2025  
**Geliştirici**: Ömer Faruk Çelik — Fırat Üniversitesi, Otonom Sistemler & Yapay Zeka Laboratuvarı



