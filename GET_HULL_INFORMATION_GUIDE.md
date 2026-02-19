# 🎯 Get Hull Information - Kapsamlı Hull + Formasyon + Grup Bilgisi

> **Fırat-GNC v1.8+** | Hull merkezi, sampled points, formasyon parametreleri ve tüm ROV grubu bilgilerini tek fonksiyon ile al

---

## 📋 Genel Bakış

`get_hull_information()` fonksiyonu hull'dan sampled points alırken, aynı zamanda:
- ✅ Hull merkezi (center point)
- ✅ Formasyon seçimi ve parametreleri (formation ID, spacing, yaw)
- ✅ Lider ROV bilgisi (ID, yaw)
- ✅ Tüm grup ROV'ları detaylı bilgileriyle (battery, mode, GPS signal, GNC capabilities)

Tek çağrıda tüm bu bilgileri **JSON-serializable** format'ta döner.

---

## 🚀 Hızlı Başlangıç

```python
# main.py'de çalışırken → Tab tuşu → Python REPL

# Temel kullanım (50 sample nokta - default)
info = filo.get_hull_information()
print(f"Hull center: {info['hull_center']}")
print(f"Sample noktalar: {len(info['hull_samples'])}")
print(f"ROV sayısı: {info['grup_bilgisi']['rov_sayisi']}")

# 100 örnek nokta iste
info = filo.get_hull_information(sample_count=100)

# Meta bilgileri göster
filo.hull_information_summary()

# JSON dosyasına kaydet
filo.hull_information_export('my_hull_info.json')
```

---

## 📖 Detaylı Fonksiyon Açıklaması

### `filo.get_hull_information(sample_count=50, g_id=0)`

**Parametreler:**
| Param | Type | Default | Açıklama |
|-------|------|---------|----------|
| `sample_count` | int | 50 | Hull çevresi üzerine kaç örnek nokta yerleştirilecek |
| `g_id` | int | 0 | Grup ID (çok gruplu sistemlerde) |

**Dönüş Değeri (Dict):**
```python
{
    # Hull Bilgisi
    'hull_center': [x, y],                           # Hull merkez koordinatları
    'hull_samples': [[x1,y1], [x2,y2], ...],         # sample_count kadar nokta
    'sample_count': 50,                              # Kullanılan örnek sayısı
    
    # Formasyon Bilgisi
    'formasyon_id': 'LINE',                          # 'LINE', 'CIRCLE', 'TRIANGLE', vb.
    'formasyon_aralik': 15.2,                        # ROV'lar arası mesafe (m)
    'formasyon_merkez': [x, y],                      # Formasyon merkez koordinatları
    'formasyon_yaw': 45.0,                           # Formasyon yönü (derece)
    
    # Lider ROV Bilgisi
    'lider_rov_id': 0,                               # Lider ROV ID
    'lider_yaw': 90.0,                               # Lider yönü (derece)
    
    # Grup Bilgisi
    'grup_id': 0,                                    # Grup ID
    'grup_bilgisi': {                                # Grup detayları
        'group_id': 0,
        'rov_sayisi': 6,                             # Grup'taki ROV sayısı
        'rov_idleri': [0, 1, 2, 3, 4, 5],
        'lider_id': 0,
        'toplam_batarya': 5.7,                       # Tüm ROV pil toplamı
        'ortalama_batarya': 0.95,                    # Ortalama pil yüzdesi (0-1)
        'rovlar': [                                  # Her ROV'un detayları
            {
                'rov_id': 0,
                'group_id': 0,
                'pozisyon': {
                    'x': 100.2,                      # X koordinatı
                    'y': 50.1,                       # Y koordinatı
                    'z': -5.0                        # Derinlik
                },
                'batarya': 0.98,                     # Pil yüzdesi (0-1)
                'rol': 1,                            # 0=normal, 1=lider
                'yaw': 45.0,                         # X-Y düzleminde yönelim (derece)
                'sonar': 2.5,                        # Son sonar mesafesi (m)
                'lidar': {'0': 12.5, '1': -1.0, ...}, # Lidar sensörü verileri
                'gnc_mode': 1,                       # GNC modu (1=aktif)
                'gps_sinyal': 1                      # GPS sinyali (1=mevcut, 0=yok)
            },
            {...},
            ...
        ]
    },
    
    # Meta Bilgi
    'timestamp': '2025-02-19 10:30:45'              # Fonksiyon çalıştırılma zamanı
}
```

---

## 📊 Kullanım Örnekleri

### Örnek 1: Hull Samples al ve Analiz et

```python
# Hull bilgisini çek
info = filo.get_hull_information(sample_count=100)

# Hull üzerine dağıtılan noktaları görüntüle
print(f"✓ Hull center: ({info['hull_center'][0]:.1f}, {info['hull_center'][1]:.1f})")
print(f"✓ Sample noktalar (ilk 3): {info['hull_samples'][:3]}")

# CSV dosyasına kaydet (sadece hull samples)
filo.hull_samples_export_csv('hull_points.csv')
```

**Çıktı:**
```
✓ Hull center: (150.2, 200.5)
✓ Sample noktalar (ilk 3): [[150.5, 200.2], [151.1, 205.3], [152.0, 210.1]]
✅ 100 nokta 'hull_points.csv' dosyasına kaydedildi
```

---

### Örnek 2: Formasyon ve Lider Bilgisi

```python
info = filo.get_hull_information()

# Formasyon detayları
print(f"Formasyon Tipi: {info['formasyon_id']}")
print(f"ROV arası mesafe: {info['formasyon_aralik']} m")
print(f"Formasyon merkezi: {info['formasyon_merkez']}")
print(f"Formasyon yönü: {info['formasyon_yaw']} °")

# Lider ROV
print(f"\nLider ROV: {info['lider_rov_id']}")
print(f"Lider yönü: {info['lider_yaw']} °")
```

**Çıktı:**
```
Formasyon Tipi: LINE
ROV arası mesafe: 15.2 m
Formasyon merkezi: [155.1, 205.2]
Formasyon yönü: 45.0 °

Lider ROV: 0
Lider yönü: 90.0 °
```

---

### Örnek 3: Grup ROV'larının Analizi

```python
info = filo.get_hull_information()

# Grup bilgisi
grup = info['grup_bilgisi']
print(f"Grup ID: {info['grup_id']}")
print(f"ROV sayısı: {grup['rov_sayisi']}")
print(f"ROV ID'leri: {grup['rov_idleri']}")
print(f"Ortalama pil: {grup['ortalama_batarya']*100:.1f}%")

# Her ROV'un detayları
for rov_data in grup['rovlar']:
    x, y, z = rov_data['pozisyon'].values()
    batarya = rov_data['batarya'] * 100
    rol = "👑 LIDER" if rov_data['rol'] == 1 else "🤖 ROV"
    print(f"{rol} {rov_data['rov_id']:>2}: Pos=({x:>6.1f},{y:>6.1f},{z:>5.1f})  Bat={batarya:>5.1f}%  Mode={rov_data['gnc_mode']}")
```

**Çıktı:**
```
Grup ID: 0
ROV sayısı: 6
ROV ID'leri: [0, 1, 2, 3, 4, 5]
Ortalama pil: 95.2%

👑 LIDER  0: Pos=(100.2, 50.1, -5.0)  Bat= 98.0%  Mode=1
🤖 ROV  1: Pos=(110.2, 55.1, -5.2)  Bat= 97.0%  Mode=1
🤖 ROV  2: Pos=(120.2, 60.1, -5.1)  Bat= 94.0%  Mode=1
🤖 ROV  3: Pos=(105.2, 65.1, -5.3)  Bat= 96.0%  Mode=1
🤖 ROV  4: Pos=(115.2, 70.1, -5.0)  Bat= 92.0%  Mode=1
🤖 ROV  5: Pos=(125.2, 75.1, -5.2)  Bat= 93.0%  Mode=1
```

---

### Örnek 4: JSON Dosyasına Kaydet ve Daha Sonra Yükle

```python
# 1. Get hull information ve JSON'a kaydet
info = filo.get_hull_information(sample_count=50)
filo.hull_information_export('mission_hull_data.json', pretty=True)

# 2. JSON'u yükle ve kullan
import json
with open('mission_hull_data.json', 'r') as f:
    saved_info = json.load(f)

print(f"Kaydedilen hull merkezi: {saved_info['hull_center']}")
print(f"Kaydedilen formasyon: {saved_info['formasyon_id']}")
```

---

### Örnek 5: Özet Bilgi Göster

```python
# Tüm bilgilerin ekrana yazdırılmış özetini gör
filo.hull_information_summary()
```

**Çıktı:**
```
==================================================
🎯 HULL INFORMATION SUMMARY
==================================================
⏱️  Timestamp: 2025-02-19 10:30:45
🎯 Hull Center: (150.20, 200.50)
📍 Hull Sample Points: 50
🔄 Formation: LINE (aralik=15.2m)
🔄 Formation Center: (155.10, 205.20)
📐 Formation Yaw: 45.0°
👑 Leader ROV: 0 (Yaw: 90.0°)
🏘️  Group ID: 0
🤖 ROV Count: 6
🔋 Average Battery: 95.2%
📊 ROV IDs: [0, 1, 2, 3, 4, 5]

ROV  Pos (x,y,z)          Battery Role   Mode GPS
------------------------------------------------------------
  0 (100.2, 50.1, -5.0)    98.0% LIDER   1  YES
  1 (110.2, 55.1, -5.2)    97.0% ROV     1  YES
  2 (120.2, 60.1, -5.1)    94.0% ROV     1  YES
  3 (105.2, 65.1, -5.3)    96.0% ROV     1  YES
  4 (115.2, 70.1, -5.0)    92.0% ROV     1  YES
  5 (125.2, 75.1, -5.2)    93.0% ROV     1  YES
==================================================
```

---

## 🔧 İleri Kullanım

### Toplu (Batch) işlem

```python
# Farklı sample count'larla birden çok analiz yap
for sample_count in [25, 50, 100]:
    info = filo.get_hull_information(sample_count=sample_count)
    print(f"\nSample count = {sample_count}")
    print(f"  - Hull samples: {len(info['hull_samples'])}")
    print(f"  - Formation: {info['formasyon_id']} ({info['formasyon_aralik']}m)")
    filo.hull_information_export(f'hull_info_{sample_count}.json')
```

---

### Cache'den Okuma (Hesaplama olmadan)

```python
# Hesapla ve cache'e kaydet
info = filo.get_hull_information()

# Daha sonra cache'den oku (çabuk)
cached_info = filo.hull_information_info()
print(cached_info['hull_center'])                    # Hızlı
```

---

### Nur Grup Bilgisi İste

```python
# Tüm get_hull_information çalıştırmadan sadece grup bilgisi al
grup_info = filo.helper.grup_bilgisi_al(group_id=0)
print(f"ROV sayısı: {grup_info['rov_sayisi']}")
for rov in grup_info['rovlar']:
    print(f"  ROV {rov['rov_id']}: Batarya=%{rov['batarya']*100:.1f}")
```

---

## 🎯 Kullanım Senaryoları

### 1️⃣ Missyon Planlaması
```python
# Missyon öncesi hull analizi ve grup durumu kontrol
info = filo.get_hull_information(sample_count=100)
print(f"Average battery ready for mission: {info['grup_bilgisi']['ortalama_batarya']*100:.1f}%")
```

### 2️⃣ Gerçek Zamanlı Harita Oluşturma
```python
# Hull noktalarını CSV'ye kaydet → Harita uygulamasında vizualize et
filo.hull_samples_export_csv('realtime_map.csv')
```

### 3️⃣ Veritabanı Kaydı
```python
# Hull ve grup bilgilerini veritabanına kaydet
info = filo.get_hull_information()
filo.hull_information_export(f'mission_{timestamp}.json')
```

### 4️⃣ AI/ML Eğitim Veri Üretimi
```python
# Hull geometry + formation + ROV states = Training sample
for i in range(100):
    info = filo.get_hull_information(sample_count=50)
    # Training veri seti oluştur
    training_sample = {
        'hull': info['hull_samples'],
        'formation': info['formasyon_id'],
        'rov_states': [r['batarya'] for r in info['grup_bilgisi']['rovlar']]
    }
    training_data.append(training_sample)
```

---

## ⚙️ İç Yapı

### Veri Akışı
```
get_hull_information()
    ├─ Formasyon seçimi (_formasyon_sec_impl)
    │   ├─ Hull hesaplama (yeni_hull)
    │   └─ Lider tespiti (find_leader_info)
    │
    ├─ get_100_samples (Hull sampled points)
    │   └─ Perimeter interpolation
    │
    ├─ grup_bilgisi_al (Grup ROV detayları)
    │   ├─ g_rovs'dan grup al
    │   └─ Her ROV için:
    │       ├─ Pozisyon
    │       ├─ Battery
    │       ├─ GNC mode & GPS signal
    │       └─ Sensors
    │
    └─ Cache'e kaydet (JSON-serializable)
        └─ Return result
```

---

## 🐛 Troubleshooting

| Problem | Çözüm |
|---------|-------|
| `❌ Hull information başarısız` | Formasyon seçimi başarısız olabilir - `formasyon_sec` debug edin |
| `❌ Cache'de hull_information yok` | Önce `get_hull_information()` çalıştırın |
| `None` dönüyor | ortam_ref, g_rovs kontrolü yapın |
| JSON export hatası | Dosya yazma izni kontrol edin |

---

## 📚 İlişkili Fonksiyonlar

- `filo.get_hull_100_samples()` - Sadece hull samples al
- `filo.hull_samples_info()` - Hull samples meta bilgisi
- `filo.formasyon_sec()` - Formasyon seçimi
- `filo.helper.grup_bilgisi_al(group_id)` - Nur grup bilgisi

---

## 💾 Cache Sistemi

```python
# Tüm cache'lenen sonuçlar:
filo.helper.last_hull_information          # get_hull_information sonucu
filo.helper.hull_information_timestamp     # Hesaplandığı zaman

# Ek hull samples cache:
filo.helper.last_hull_samples              # 100 samples
filo.helper.hull_samples_timestamp         # Hesaplandığı zaman
```

---

**Last Updated**: Feb 2025  
**Status**: ✅ Production Ready  
**Version**: 1.0  

*See also: [HULL_SAMPLES_GUIDE.md](HULL_SAMPLES_GUIDE.md), [FORMASYON_ASYNC_GUIDE.md](FORMASYON_ASYNC_GUIDE.md)*
