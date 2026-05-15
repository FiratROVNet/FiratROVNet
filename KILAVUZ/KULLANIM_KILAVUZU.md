# 🚢 FiratROVNet Kullanım Kılavuzu

FiratROVNet, çoklu ROV (Remotely Operated Vehicle) simülasyonu ve yönetimi için geliştirilmiş bir Python kütüphanesidir. Güdüm, Navigasyon ve Kontrol (GNC), kamera yönetimi, GAT tabanlı analiz ve grup bazlı navigasyon kuyruğu destekler.

---

## 📦 Kurulum

### Gereksinimler

```bash
pip install torch torch_geometric ursina numpy networkx scipy
```

**Ana Bağımlılıklar:**
- `torch` – Derin öğrenme (GAT)
- `torch-geometric` – Graf sinir ağları
- `ursina` – 3D simülasyon ve fizik
- `numpy`, `scipy` – Hesaplama ve Convex Hull
- `networkx` – Graf işlemleri

---

## 🚀 Hızlı Başlangıç

### Ana Uygulama ile Simülasyon

```python
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo

# Ortam ve simülasyon
app = Ortam()
app.sim_olustur(
    n_rovs=(4, 3),           # Grup başına ROV sayıları (toplam 7)
    n_islands=4,
    havuz_genisligi=200,
    rov_model='submarine'    # veya 'bluerov2'
)

# Filo, ortama referans ile oluşturulur (GNC, motor, kamera otomatik kurulur)
filo = Filo(ortam_ref=app)

# Simülasyonu çalıştır (konsol interaktif)
app.run(interaktif=True)
```

`Filo(ortam_ref=app)` verildiğinde fizik gövdeleri, motorlar, minimap ve varsayılan kamera otomatik ayarlanır.

---

## 📋 Filo Sınıfı - Ana API

### Temel Kullanım

```python
from FiratROVNet.gnc import Filo

# Ortam referansı ile (önerilen — main.py ile uyumlu)
filo = Filo(ortam_ref=app)
```

### 1. Simülasyon Oluşturma — `sim_olustur()`

Ortam nesnesi üzerinde çağrılır:

```python
app.sim_olustur(
    n_rovs=(4, 3),           # Tuple: her eleman bir gruptaki ROV sayısı
    n_islands=5,             # Ada sayısı
    n_rocks=20,              # Kaya/engel sayısı (opsiyonel)
    havuz_genisligi=200,     # Havuz yarı genişliği (metre)
    rov_model='submarine'    # 'submarine' | 'bluerov2'
)
```

### 2. Hedef Atama — `git()`

ROV’a hedef koordinatı atar (konsolda `git(rov_id, x, z, y=None, ai=True)` olarak kullanılır):

```python
# ROV-0'ı (x=50, z=50, derinlik=-5) koordinatına gönder
filo.git(0, 50, 50, -5)

# ROV-1'i aynı şekilde, AI kapalı (kör mod)
filo.git(1, -20, 100, -10, ai=False)

# Sadece yatay koordinat (y/derinlik opsiyonel)
filo.git(2, 30, 40)
```

**Parametreler:**
- `rov_id`: ROV indeksi (0, 1, 2, …)
- `x`, `z`: Yatay düzlem koordinatları
- `y`: Derinlik (opsiyonel; negatif = su altı)
- `ai`: `True` = zeki mod (GAT/APF), `False` = kör mod

**Koordinat Sistemi:**
- `x`, `z`: Yatay düzlem (sağ-sol, ileri-geri)
- `y`: Derinlik (negatif = su altı)

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

### 4. Manuel Hareket — `move()`

ROV’a güç bazlı hareket komutu verir (konsolda `move(rov_id, yon, guc=1.0)`):

```python
filo.move(0, 'ileri', 1.0)   # ROV-0 %100 ileri
filo.move(1, 'sag', 0.5)     # ROV-1 %50 sağa
filo.move(2, 'cik', 0.3)     # ROV-2 %30 yukarı
filo.move(3, 'dur', 0.0)     # ROV-3 dur
```

**Yönler:** `'ileri'`, `'geri'`, `'sag'`, `'sol'`, `'cik'`, `'bat'`, `'dur'`  
**Güç:** `0.0`–`1.0` (örn. `1.0` = %100).

### 5. ROV Ayarları — `set()`

ROV ayarlarını değiştirir (konsolda `set(rov_id, ayar_adi, deger)`):

```python
filo.set(0, 'rol', 1)                    # ROV-0 lider
filo.set(1, 'engel_mesafesi', 30.0)      # Engel menzili (m)
filo.set(2, 'iletisim_menzili', 80.0)    # İletişim menzili (m)
```

**Ayar örnekleri:** `'rol'`, `'renk'`, `'engel_mesafesi'`, `'iletisim_menzili'`, `'min_pil_uyarisi'` vb.

### 6. ROV Bilgisi — `get()`

ROV verilerini okur (konsolda `get(rov_id, veri_tipi)`):

```python
pozisyon = filo.get(0, 'gps')      # (x, y, z)
batarya = filo.get(0, 'batarya')
rol = filo.get(1, 'rol')           # 1=Lider, 0=Takipçi
sensörler = filo.get(2, 'sensör')
```

**Veri tipleri:** `'gps'`, `'hiz'`, `'batarya'`, `'rol'`, `'renk'`, `'sensör'`, `'sonar'` vb.

### 7. Kamera Yönetimi

Filo, `camera_manager` ile ROV FPV kameralarını yönetir:

```python
# Belirli bir ROV'un kamerasını aktif et (önceki kameralar kapatılır)
filo.kamera_ayarla(rov_id=1)

# Kamera kaldır
filo.kamera_kaldir(rov_id=1)
```

Oyun içi **R** tuşu, bilgi paneli ve takip kamerasını sıradaki ROV’a geçirir; bu da dahili olarak `filo.kamera_ayarla(rov_id=...)` kullanır.

### 8. Navigasyon Kuyruğu

Minimap’e tıklanarak eklenen hedefler grup bazlı `nav_queue` içinde tutulur:

```python
# filo.nav_queue = { g_id: [ {'pos': (x,y,z), 'id': n}, ... ] }
# Konsoldan izlemek için: nav_queue
```

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
from FiratROVNet.model_paths import GAT_MODEL, path_str
from GAT.gat_test import FiratAnalizci

# Model yükle
beyin = FiratAnalizci(model_yolu=path_str(GAT_MODEL))

# Veri analiz et
veri = app.get_gat_data()
tahminler, _, _ = beyin.analiz_et(veri)

# Tahminleri kullan
filo.guncelle_hepsi(tahminler)
```

---

## 🎮 Konsol Erişimi

Simülasyon `interaktif=True` ile çalışırken terminalde Python kabuğu (`>>>`) açık kalır. `main.py` aşağıdaki fonksiyonları ve nesneleri konsola ekler:

- **`git(rov_id, x, z, y=None, ai=True)`** — Hedef atama
- **`move(rov_id, yon, guc=1.0)`** — Manuel hareket
- **`get(rov_id, veri_tipi)`** — Veri okuma
- **`set(rov_id, ayar_adi, deger)`** — Ayar değiştirme
- **`Ada(ada_id, x=None, y=None)`**, **`ROV(rov_id, x=None, y=None, z=None)`** — Ortam nesneleri
- **`filo`**, **`rovs`**, **`cfg`**, **`nav_queue`** — Referanslar

```python
>>> git(0, 50, 70, -5)
>>> move(0, "ileri", 1.0)
>>> get(0, 'gps')
>>> set(0, 'engel_mesafesi', 50.0)
>>> filo.kamera_ayarla(rov_id=1)
```

---

## 💡 Örnek Kullanım Senaryoları

### Senaryo 1: Basit Kurulum

```python
from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo

app = Ortam()
app.sim_olustur(n_rovs=(4, 3), n_islands=4, havuz_genisligi=200, rov_model='submarine')
filo = Filo(ortam_ref=app)
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
from FiratROVNet.model_paths import GAT_MODEL, path_str
from GAT.gat_test import FiratAnalizci

beyin = FiratAnalizci(model_yolu=path_str(GAT_MODEL))

def update():
    veri = app.get_gat_data()
    tahminler, _, _ = beyin.analiz_et(veri)
    filo.guncelle_hepsi(tahminler)

app.set_update_function(update)
app.run(interaktif=True)
```

### Senaryo 4: Manuel Kontrol

```python
filo.move(0, 'ileri', 1.0)
filo.move(1, 'sag', 0.5)
filo.move(2, 'cik', 0.3)
filo.move(3, 'dur', 0.0)
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

`main.py` zaten `filo`, `rovs`, `cfg`, `git`, `move`, `get`, `set`, `Ada`, `ROV`, `nav_queue` ekler. Konsol mesajı 1–2 saniye sonra görünür; terminali kontrol edin.

---

## 📚 İlgili Dokümantasyon

- **Senaryo Modülü**: `KILAVUZ/SENARYO_KULLANIM.md`
- **Konsol Erişimi**: `KILAVUZ/KONSOL_ERISIM.md`

---

## 👨‍💻 Geliştirici

Ömer Faruk Çelik — Fırat Üniversitesi, Otonom Sistemler & Yapay Zeka Laboratuvarı

---

## 📝 Notlar

- `Filo(ortam_ref=app)` ile ortam verildiğinde GNC, motor ve kamera otomatik kurulur.
- `guncelle_hepsi()` ana döngüde (update) her frame çağrılmalıdır.
- Koordinatlar: yatay `x`, `z`; derinlik `y` (negatif = su altı).
- Kamera: **R** tuşu veya `filo.kamera_ayarla(rov_id=...)`; minimap tıklama ile hedef kuyruğa eklenir.
