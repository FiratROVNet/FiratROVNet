# 🚢 Filo() Sınıfı Kullanım Kılavuzu

## Temel Kullanım

```python
from FiratROVNet.gnc import Filo

filo = Filo()
```

---

## 📋 Ana Metodlar

### 1. **`otomatik_kurulum()`** - Otomatik Sistem Kurulumu
Tüm ROV'ları otomatik olarak kurar (modem, GNC, sensör, hedefler)

```python
tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,                    # ROV listesi
    lider_id=0,                       # Lider ROV ID
    modem_ayarlari={...},             # Modem parametreleri
    baslangic_hedefleri={...},        # Başlangıç hedefleri
    sensor_ayarlari={...}             # Sensör ayarları
)
```

**Özellikler:**
- ✅ Otomatik modem oluşturma (lider/takipçi için ayrı)
- ✅ Otomatik GNC sistemi kurulumu
- ✅ Sensör ayarları yapılandırma
- ✅ Başlangıç hedefleri atama
- ✅ Modem rehberi otomatik dağıtma

---

### 2. **`git(rov_id, x, z, y=None, ai=True)`** - Hedef Atama
Belirtilen ROV'a hedef koordinatı atar

```python
filo.git(0, 40, 60, 0)           # ROV-0: (40, 60, 0)
filo.git(1, 35, 50, -10, ai=False)  # ROV-1: AI kapalı
```

**Parametreler:**
- `rov_id`: ROV ID (0, 1, 2, ...)
- `x, z, y`: Hedef koordinatları (y=derinlik, opsiyonel)
- `ai`: AI aktif/pasif (varsayılan: True)

---

### 3. **`guncelle_hepsi(tahminler)`** - Toplu Güncelleme
Tüm ROV'ları GAT tahminleriyle günceller

```python
tahminler = [0, 1, 0, 2]  # Her ROV için GAT kodu
filo.guncelle_hepsi(tahminler)
```

**GAT Kodları:**
- `0`: OK (Normal)
- `1`: ENGEL (Engel tespit edildi)
- `2`: CARPISMA (Çarpışma riski)
- `3`: KOPUK (Bağlantı koptu)
- `5`: UZAK (Liderden uzak)

---

### 4. **`move(rov_id, yon, guc=1.0)`** - Güç Bazlı Manuel Hareket
ROV'a güç bazlı hareket komutu verir (gerçek dünya gibi motor gücü kontrolü)

```python
filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
filo.move(2, 'cik', 0.3)     # ROV-2 %30 güçle yukarı
filo.move(3, 'dur', 0.0)     # ROV-3 dur (güç=0)
filo.move(0, 'ileri')        # ROV-0 %100 güçle ileri (varsayılan)
```

**Yönler:**
- `'ileri'`, `'geri'`, `'sag'`, `'sol'`, `'cik'`, `'bat'`, `'dur'`

**Güç Parametresi:**
- `1.0` = %100 güç (maksimum hız)
- `0.5` = %50 güç (yarı hız)
- `0.3` = %30 güç (düşük hız)
- `0.0` = %0 güç (dur)

**Özellikler:**
- ✅ Gerçek dünya gibi motor gücü kontrolü
- ✅ Havuz sınır kontrolü otomatik (sınırda hareket engellenir)
- ✅ Lider ROV batırılamaz kontrolü
- ✅ Güç değeri 0.0-1.0 arası otomatik sınırlanır
- ✅ Sürekli hareket (her frame güç uygulanır)

---

### 5. **`set(rov_id, ayar_adi, deger)`** - ROV Ayarı
ROV ayarlarını değiştirir

```python
filo.set(0, 'rol', 1)                    # ROV-0'ı lider yap
filo.set(1, 'renk', (255, 0, 0))        # ROV-1'i kırmızı yap
filo.set(2, 'engel_mesafesi', 30.0)      # Sensör ayarı
```

**Desteklenen ayarlar:**
- `'rol'`: Lider (1) veya Takipçi (0)
- `'renk'`: RGB tuple `(r, g, b)` veya renk ismi
- `'engel_mesafesi'`, `'iletisim_menzili'`, `'min_pil_uyarisi'`

---

### 6. **`get(rov_id, veri_tipi)`** - ROV Bilgisi
ROV bilgilerini alır

```python
pozisyon = filo.get(0, 'gps')
rol = filo.get(1, 'rol')
sensörler = filo.get(2, 'sensör')
renk = filo.get(0, 'renk')
```

**Desteklenen veri tipleri:**
- `'gps'`, `'hiz'`, `'batarya'`, `'rol'`, `'renk'`
- `'sensör'`, `'engel_mesafesi'`, `'iletisim_menzili'`, `'min_pil_uyarisi'`, `'sonar'`

---

### 7. **`ekle(gnc_objesi)`** - Manuel GNC Ekleme
Manuel olarak GNC sistemi ekler (otomatik kurulum yerine)

```python
gnc = LiderGNC(rov, modem)
filo.ekle(gnc)
```

---

### 8. **`rehber_dagit(modem_rehberi)`** - Modem Rehberi
Lider ROV'a modem rehberi dağıtır (otomatik kurulumda otomatik yapılır)

```python
tum_modemler = {0: modem0, 1: modem1, 2: modem2}
filo.rehber_dagit(tum_modemler)
```

---

## 🔧 Özellikler

### Erişim
```python
filo.sistemler  # Tüm GNC sistemlerine erişim
len(filo.sistemler)  # ROV sayısı
```

### Otomatik Sistemler

**Havuz Sınır Kontrolü:**
- ROV'lar havuz dışına çıkamaz
- X, Z eksenlerinde otomatik sınır kontrolü
- Sınırda hız otomatik sıfırlanır

**Çarpışma Sistemi:**
- ROV-ROV çarpışması: Momentum korunumu ile gerçekçi çarpışma
- ROV-Kaya çarpışması: Yansıma fiziği
- Otomatik pozisyon ayrımı

**Lider ROV Özellikleri:**
- Otomatik su yüzeyine çıkar
- Batırılamaz (bat komutu işe yaramaz)
- Rol değiştirilince batırılabilir olur
```

### Örnek Kullanım Senaryoları

**Senaryo 1: Basit Kurulum**
```python
filo = Filo()
filo.otomatik_kurulum(rovs=app.rovs)
```

**Senaryo 2: Özel Ayarlarla**
```python
filo = Filo()
filo.otomatik_kurulum(
    rovs=app.rovs,
    lider_id=0,
    baslangic_hedefleri={0: (40, 0, 60), 1: (35, -10, 50)},
    sensor_ayarlari={'engel_mesafesi': 25.0, 'iletisim_menzili': 40.0}
)
```

**Senaryo 3: Dinamik Hedef Değiştirme**
```python
# Simülasyon sırasında hedef değiştir
filo.git(0, 50, 70, -5)  # Lider yeni hedefe
filo.git(1, 45, 65, -10)  # Takipçi yeni hedefe
```

**Senaryo 4: AI Kontrolü**
```python
filo.git(0, 40, 60, 0, ai=True)   # AI açık
filo.git(1, 35, 50, -10, ai=False) # AI kapalı (kör mod)
```

**Senaryo 5: Güç Bazlı Manuel Hareket**
```python
# ROV'ları güç bazlı manuel kontrol et
filo.move(0, 'ileri', 1.0)   # ROV-0 %100 güçle ileri
filo.move(1, 'sag', 0.5)     # ROV-1 %50 güçle sağa
filo.move(2, 'cik', 0.3)     # ROV-2 %30 güçle yukarı
filo.move(3, 'dur', 0.0)     # ROV-3 dur

# Yavaş hareket
filo.move(0, 'ileri', 0.2)   # ROV-0 %20 güçle yavaş ileri
filo.move(1, 'geri', 0.1)    # ROV-1 %10 güçle çok yavaş geri
```

**Senaryo 6: ROV Ayarları**
```python
# Lider değiştir
filo.set(2, 'rol', 1)  # ROV-2 lider olur, otomatik su yüzeyine çıkar

# Renk değiştir
filo.set(0, 'renk', (0, 255, 0))  # ROV-0 yeşil
filo.set(1, 'renk', 'mavi')       # ROV-1 mavi (renk ismi ile)

# Sensör ayarları
filo.set(0, 'engel_mesafesi', 30.0)
filo.set(1, 'iletisim_menzili', 50.0)

# Bilgi al
pozisyon = filo.get(0, 'gps')
rol = filo.get(0, 'rol')
tum_sensorler = filo.get(0, 'sensör')
```

---

## 💡 İpuçları

1. **Otomatik kurulum** kullanın - Manuel kurulumdan daha kolay
2. **Sensör ayarları** ile her ROV'u özelleştirebilirsiniz
3. **Modem ayarları** ile iletişim kalitesini simüle edin
4. **`guncelle_hepsi()`** her frame'de çağrılmalı (update döngüsünde)
5. **`filo.sistemler`** ile her ROV'un GNC sistemine erişebilirsiniz
6. **`move()` komutu** ile manuel kontrol yapabilirsiniz (havuz sınırları otomatik)
7. **Lider ROV** otomatik su yüzeyinde kalır ve batırılamaz
8. **Çarpışmalar** otomatik işlenir (momentum korunumu)
9. **`set()` ve `get()`** ile ROV'ları dinamik olarak yönetin
10. **Renk ayarları** RGB tuple veya renk ismi ile yapılabilir

---

## 📝 Notlar

- `otomatik_kurulum()` çağrıldığında tüm ayarlar otomatik yapılır
- Manuel `ekle()` kullanırsanız `rehber_dagit()` manuel çağrılmalı
- `git()` metodu AI durumunu da kontrol eder
- Tüm ROV'lar `filo.sistemler` listesinde saklanır
- `move()` komutu havuz sınırlarını otomatik kontrol eder
- Lider ROV (`rol=1`) batırılamaz, otomatik su yüzeyinde kalır
- Çarpışmalar her frame'de otomatik kontrol edilir (momentum korunumu)
- `set('rol', 0)` ile lider takipçiye dönüştürülebilir (artık batırılabilir)
- Renk ayarları: RGB tuple `(r, g, b)` veya renk ismi (`'kirmizi'`, `'mavi'`, vb.)

