# 🔋 Batarya Sistemi

## 📋 Genel Bakış

ROV'lar artık gerçekçi bir batarya sistemi ile çalışır. Batarya tüketimi hareket gücüne bağlıdır ve batarya bitince ROV'lar hareket edemez ve yüzeye çıkar.

---

## ⚙️ Batarya Tüketim Formülü

```python
batarya = batarya - gecen_sure * rov_calistirilan_guc * somurme_katsayisi
```

### **Parametreler:**

- **`gecen_sure`**: `time.dt` (frame başına geçen süre)
- **`rov_calistirilan_guc`**: ROV'un çalıştırdığı güç (0.0-1.0)
  - Hız bazlı: `velocity.length() / 100.0`
  - Duruyorsa: `0.0` (batarya tüketimi yok)
- **`somurme_katsayisi`**: `BATARYA_SOMURME_KATSAYISI = 0.01` (küçük değer, batarya yavaş bitsin)

---

## 🎯 Batarya Durumları

### **1. Normal Çalışma (Batarya > 0):**
- ROV normal şekilde hareket eder
- Batarya tüketimi hız ve güce bağlı
- Duruyorsa batarya tüketimi yok

### **2. Batarya Bitti (Batarya <= 0):**
- ✅ **Hareket Engellenir:** ROV hareket edemez
- ✅ **Yüzeye Çıkar:** Otomatik olarak su yüzeyine çıkar
- ✅ **Renk Değişir:** Gri renge döner (`color.rgb(100, 100, 100)`)
- ✅ **Sürüden Ayrılır:** Manuel kontrol açılır (GNC devre dışı)
- ✅ **Label Güncellenir:** "🔋BİTTİ" gösterilir

---

## 📊 Batarya Tüketim Örnekleri

### **Örnek 1: Duruyor (Güç = 0.0)**
```python
# ROV duruyor, batarya tüketimi yok
calistirilan_guc = 0.0
batarya_tuketimi = time.dt * 0.0 * 0.01 = 0.0
# Batarya değişmez
```

### **Örnek 2: Yavaş Hareket (Güç = 0.3)**
```python
# ROV yavaş hareket ediyor
calistirilan_guc = 0.3
batarya_tuketimi = time.dt * 0.3 * 0.01 = 0.003 * time.dt
# Saniyede ~0.3% batarya tüketimi
```

### **Örnek 3: Maksimum Hız (Güç = 1.0)**
```python
# ROV maksimum hızda hareket ediyor
calistirilan_guc = 1.0
batarya_tuketimi = time.dt * 1.0 * 0.01 = 0.01 * time.dt
# Saniyede ~1% batarya tüketimi
```

---

## 🔧 Kod Yapısı

### **ROV.update() - Batarya Tüketimi:**

```python
# Batarya tüketimi (gerçekçi fizik)
if self.battery > 0:
    # Çalıştırılan güç hesapla (hız ve hareket durumuna göre)
    mevcut_guc = abs(self.velocity.length()) / 100.0  # 0.0-1.0 arası normalize
    if mevcut_guc > 0.01:  # Hareket varsa
        self.calistirilan_guc = mevcut_guc
        # Batarya tüketimi: batarya = batarya - gecen_sure * rov_calistirilan_guc * somurme_katsayisi
        self.battery -= time.dt * self.calistirilan_guc * BATARYA_SOMURME_KATSAYISI
        self.battery = max(0.0, self.battery)  # Negatif olamaz
    else:
        self.calistirilan_guc = 0.0  # Duruyorsa güç tüketimi yok
```

### **Batarya Bitme Kontrolü:**

```python
# Batarya bitti mi kontrol et
if self.battery <= 0 and not self.batarya_bitti:
    self.batarya_bitti = True
    # Manuel kontrolü aç (sürüden ayrıl)
    # Yüzeye çık
    self.velocity = Vec3(0, 0, 0)
    # Renk değiştir (batarya bitti rengi)
    self.color = color.rgb(100, 100, 100)  # Gri
    print(f"🔋 [ROV-{self.id}] Batarya bitti! Yüzeye çıkıyor...")
```

### **Batarya Bitmişse Hareket Engelleme:**

```python
# Batarya bitmişse hareket ettirme
if self.batarya_bitti:
    # Sadece yüzeye çık
    if self.y < 0:
        self.velocity.y = 2.0  # Yüzeye çık
    else:
        self.velocity = Vec3(0, 0, 0)  # Yüzeyde dur
    # Manuel hareketi engelle
    if self.manuel_hareket['yon'] is not None:
        self.manuel_hareket['yon'] = None
        self.manuel_hareket['guc'] = 0.0
    return  # Batarya bitmişse diğer işlemleri yapma
```

---

## 🎨 Görsel Göstergeler

### **Label'da Batarya Bilgisi:**

```python
# Batarya bilgisi ekle
batarya_bilgisi = f"\n🔋{app.rovs[i].battery:.0f}%"
if app.rovs[i].batarya_bitti:
    batarya_bilgisi = "\n🔋BİTTİ"
app.rovs[i].label.text = f"R{i}\n{durum_txts[gat_kodu]}{mesafe_bilgisi}{batarya_bilgisi}{ek}"
```

### **Renk Değişimi:**

- **Normal:** GAT koduna göre renk
- **Batarya Bitti:** Gri (`color.rgb(100, 100, 100)`)

---

## ⚙️ Özelleştirme

### **Batarya Tüketim Katsayısını Değiştir:**

`simulasyon.py` dosyasında:

```python
BATARYA_SOMURME_KATSAYISI = 0.01  # Varsayılan: 0.01
# Daha hızlı tüketim için: 0.02
# Daha yavaş tüketim için: 0.005
```

### **Başlangıç Batarya Seviyesi:**

`ROV.__init__()` içinde:

```python
self.battery = 100.0  # Varsayılan: 100%
# Farklı başlangıç seviyesi için: 50.0, 75.0, vb.
```

---

## 📝 Özet

| Durum | Batarya | Hareket | Renk | Davranış |
|-------|---------|---------|------|----------|
| **Normal** | > 0 | ✅ Aktif | GAT koduna göre | Normal hareket |
| **Bitti** | <= 0 | ❌ Engellendi | Gri | Yüzeye çık, dur |

**Sonuç:** Batarya sistemi gerçekçi fizik kurallarıyla çalışır ve batarya bitince ROV'lar otomatik olarak yüzeye çıkar! 🔋

