# 🔹Hull 100 Samples - Console Kılavuzu

> **Özellik**: Hull'dan 100 eşit-dağıtılmış örnek nokta al ve elle işle

---

## 📦 Sistem Mimarisi

```
┌─────────────────────────────────────────┐
│ Hull (ada/engel polygon)                │
│ └─ yeni_hull() → hull_output           │
│    ├─ points: N polygon noktası         │
│    ├─ area: Hull alanı                  │
│    └─ ...meta data                      │
└─────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────┐
│ get_100_samples()                       │
│ └─ Cevre uzunluğu → 100 sample nokta  │
│    (Eşit aralıklı interpolasyon)      │
└─────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────┐
│ 📦 Cache Layer                          │
├─────────────────────────────────────────┤
│ • last_hull_samples (liste: [[x,y]...])│
│ • last_hull_samples_info (meta)         │
│ • hull_samples_timestamp                │
└─────────────────────────────────────────┘
            ↓
┌─────────────────────────────────────────┐
│ Console Helpers (main thread)           │
├─────────────────────────────────────────┤
│ • hull_samples()  → Oku cache          │
│ • hull_samples_info() → Meta           │
│ • get_hull_100_samples() → Hesapla    │
│ • hull_samples_export_csv() → Kaydet  │
└─────────────────────────────────────────┘
```

---

## 🎯 Kullanım Örnekleri

### 1️⃣ **Basit: Direkt Hesapla + Sonuca Eriş**

```python
# Konsol'dan:
>>> samples = filo.get_hull_100_samples()
✅ Hull 100 samples alındı (100 nokta)

>>> print(type(samples))
<class 'list'>

>>> print(len(samples))
100

>>> print(samples[0])  # İlk nokta
[-45.32, 123.65]

>>> print(samples[-1]) # Son nokta
[-44.98, 124.12]
```

### 2️⃣ **Meta Bilgi Oku**

```python
# Konsol'dan:
>>> filo.get_hull_100_samples()
✅ Hull 100 samples alındı (100 nokta)

>>> info = filo.hull_samples_info()
>>> print(info)
{
    'bilgi': {
        'point_count': 100,
        'is_valid': True,
        'hull_area': 753.86,
        'hull_points': 15
    },
    'zaman': 1708300500.123
}

>>> print(f"Alan: {info['bilgi']['hull_area']} m²")
Alan: 753.86 m²
```

### 3️⃣ **Tümü Bir Seferde: Formatted Gösterim**

```python
# Konsol'dan:
>>> samples = filo.get_hull_100_samples()
>>> result = filo.hull_samples(formatted=True)

📊 Shape: (100, 2)
📐 Min: [-50.12, 120.34], Max: [50.87, 230.45]

>>> print(result['info'])
{'point_count': 100, 'is_valid': True, ...}
```

### 4️⃣ **CSV'ye Kaydet**

```python
# Konsol'dan:
>>> filo.get_hull_100_samples()
>>> filo.hull_samples_export_csv('hull_points.csv')
✅ 100 nokta 'hull_points.csv' dosyasına kaydedildi

# hull_points.csv:
# x,y
# -45.32,123.65
# -44.98,124.12
# ...
```

### 5️⃣ **Batch: Birden Fazla Hull Alanından Örnek Al**

```python
# Konsol'dan:
results = {}

# Grup 0 için
filo.formasyon_sec(g_id=0, dinamik=True)
time.sleep(0.2)
samples_0 = filo.get_hull_100_samples()
results[0] = samples_0

# Grup 1 için
filo.formasyon_sec(g_id=1, dinamik=True)
time.sleep(0.2)
samples_1 = filo.get_hull_100_samples()
results[1] = samples_1

# Her ikişi de cache'te
for g_id, samples in results.items():
    print(f"Grup-{g_id}: {len(samples)} nokta, ilk nokta: {samples[0]}")
```

---

## 💻 Console Methods Referans

| Method | Açıklama | Return |
|--------|----------|--------|
| `get_hull_100_samples(hull_output=None, sample_count=100)` | Hesapla + cache + döndür | `[[x,y], ...]` |
| `hull_samples(clear=False, formatted=False)` | Cache'den oku | `{'samples': [...], 'info': {...}, 'zaman': ts}` |
| `hull_samples_info()` | Meta bilgi | `{'bilgi': {...}, 'zaman': ts}` |
| `hull_samples_export_csv(filename, clear=False)` | CSV'ye kaydet | `True/False` |

---

## 🔧 Teknik Detaylar

### Hull Points Sampling Algoritması

```
1. Hull polygon noktalarından → P1, P2, ..., PN
2. Ardışık segment uzunluğu hesapla → L1, L2, ..., L(N-1)
3. Kumulatif mesafe → cumsum(L) = [0, L1, L1+L2, ...]
4. Total çevre = cumsum[-1]
5. 100 eşit nokta çık → distances = [0, T/100, 2T/100, ..., 99T/100]
6. Linear interpolasyon → X = interp(distances, cumsum, X_coords)
7. Sonuç: 100 eşit-aralıklı nokta
```

### Cache Yapısı

```python
# FiloHelper.core içinde:
self.last_hull_samples = [[x1,y1], [x2,y2], ...]  # List (JSON-safe)
self.last_hull_samples_info = {                    # JSON-safe dict
    'point_count': 100,
    'is_valid': True,
    'hull_area': 753.86,
    'hull_points': 15
}
self.hull_samples_timestamp = 1708300500.123       # Float
```

---

## 🎓 Best Practices

1. **Batch Processing:**
   ```python
   for g_id in range(5):
       filo.formasyon_sec(g_id=g_id)
   time.sleep(1)  # Tümü bitmesi için bekle
   
   for g_id in range(5):
       samples = filo.get_hull_100_samples()
       # İşle
   ```

2. **Error Checking:**
   ```python
   samples = filo.get_hull_100_samples()
   info = filo.hull_samples_info()
   
   if info['bilgi']['is_valid']:
       print(f"✅ {len(samples)} nokta")
       area = info['bilgi']['hull_area']
   else:
       print("❌ Hull sampling başarısız")
   ```

3. **CSV Export Pipeline:**
   ```python
   filo.get_hull_100_samples()
   filo.hull_samples_export_csv('output.csv', clear=True)
   # Cache temizlenir, dosya kaydedilir
   ```

4. **Timestamp Kullan:**
   ```python
   result = filo.hull_samples()
   print(f"Son güncelleme: {result['zaman']}")
   # Bu timestamp'le eski/yeni ayırt et
   ```

---

## ⚠️ Sorun Giderme

### S: "❌ Hull sampling başarısız" hatası
```python
# A: Ada/engel listesi boş veya hull oluşturulamıyor
debug_info = filo.hull_samples_info()
print(debug_info)  # is_valid: False
# Ortam kontrol et, ada/engel ekle
```

### S: Samples'ın boş döndüğü
```python
# A: Hull henüz hesaplanmamış
samples = filo.get_hull_100_samples(hull_output=manually_calculated_hull)
# Veya formasyon_sec çağrı → sonra deneme
```

### S: CSV yazma hatası
```python
# A: Dosya yolu/izni problem
filo.hull_samples_export_csv('/tmp/hull.csv')  # Absolute path kullan
```

---

## 🔗 İlgili Metodlar

- `filo.formasyon_sec(...)` - Formasyon seç (hull referansı oluştur)
- `filo.helper.yeni_hull(...)` - Direct hull hesapla
- `filo.helper.ada_cevre()` - Ada/engel listesi al
- `filo.hull_samples()` - Cache oku

---

## 📚 Dosyalar

- **Implementation**: `FiratROVNet/kutuphane/helper/gnc_helper/mixins/training.py`
- **Cache Init**: `FiratROVNet/kutuphane/helper/gnc_helper/core.py`
- **Console Wrappers**: `FiratROVNet/gnc/__init__.py`

---

**Last Updated**: February 19, 2026  
**Version**: v1.7.7+ (Hull Samples Feature)
