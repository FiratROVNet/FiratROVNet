# 🎯 Hull 100 Samples - Quick Reference

> **Konsol'dan hızlı erişim kartı** | Use during `python main.py`

---

## 📌 Basic Usage

```python
# Tab tuşu ile Python REPL'e gireceksiniz, sonra:

# ✅ 100 örnek hesapla ve cache'e kaydet
samples = filo.get_hull_100_samples()
print(len(samples))  # 100
print(samples[:3])   # İlk 3 örnek: [[x1,y1], [x2,y2], [x3,y3]]

# ✅ Cache'ten oku (hızlı, hesaplama yok)
samples = filo.hull_samples()          # Baştaki cache
samples2 = filo.hull_samples(formatted=True)  # Güzel format

# ✅ Meta bilgi
info = filo.hull_samples_info()
print(f"Puan sayısı: {info['point_count']}")
print(f"Geçerli mi: {info['is_valid']}")
print(f"Saat: {info['timestamp']}")

# ✅ CSV'ye kaydet
filo.hull_samples_export_csv('my_hull_samples.csv')
# my_hull_samples.csv oluşturulur: x,y header ile
```

---

## 🔍 Advanced Usage

```python
# Custom 50 örnek
samples_50 = filo.get_hull_100_samples(sample_count=50)

# Özel hull_output ile
from FiratROVNet.hull import hull_calculator
hull = hull_calculator(obstacle_list)  # Örnek hull hesaplayın
samples = filo.get_hull_100_samples(hull_output=hull)

# Cache'i temizle (yeni hesaplama için)
samples = filo.hull_samples(clear=True)  # Eski cache silinir

# CSV'yi temizle ve kaydet
filo.hull_samples_export_csv('hull.csv', clear=True)
```

---

## 🎲 Toplu İşlem

```python
# 5 ayrı hull için örnek al
for i in range(5):
    samples = filo.get_hull_100_samples(sample_count=100)
    filo.hull_samples_export_csv(f'hull_sample_{i}.csv', clear=True)

# Veya loop içinde cache oku
for i in range(5):
    info = filo.hull_samples_info()
    print(f"Örnek {i}: {info['point_count']} puan")
```

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| `hull_samples()` returns None | `get_hull_100_samples()` çalıştırın önce |
| `AttributeError: filo has no attribute 'hull_samples'` | gnc/__init__.py'de import kontrol edin |
| CSV dosya oluşmadı | Write permission için folder kontrol edin |
| Sample sayısı 100 değil | `sample_count` parametresi kontrol edin |

---

## 📊 Output Examples

```python
>>> samples = filo.get_hull_100_samples()
>>> samples[:3]
[[150.2, 200.5], [151.1, 205.3], [152.0, 210.1]]

>>> info = filo.hull_samples_info()
>>> info
{
  'point_count': 100,
  'is_valid': True,
  'hull_area': 45230.5,
  'timestamp': '2025-02-14 10:30:45',
  'hull_perimeter': 325.8
}

# CSV content (hull_samples.csv):
# x,y
# 150.2,200.5
# 151.1,205.3
# ...
```

---

## 💡 Integration with Other Features

```python
# Formasyon + Hull samples
filo.formasyon_sec()         # Formation hesapla
info = filo.hull_samples_info()  # Hull cache kontrol et

# ROV hareketi + Hull export
git(0, 100, 50)              # ROV'u hareket ettir
filo.hull_samples_export_csv('path_record.csv')  # Yolu kaydet
```

---

**Last Updated**: Feb 2025  
**Status**: ✅ Production Ready  
**Tests**: 5/5 Passed

---

*Daha fazla detay için [HULL_SAMPLES_GUIDE.md](HULL_SAMPLES_GUIDE.md) veya [RL_PPO/RL_PPO_MODELS_DOCUMENTATION.py](RL_PPO/RL_PPO_MODELS_DOCUMENTATION.py) bkz.*
