# 🚀 Simülasyon Başlangıç Davranışları

## 📋 Genel Bakış

Simülasyon başladığında ROV'ların nasıl davrandığını ve hangi adımların izlendiğini açıklar.

---

## 🔄 Başlangıç Sırası

### 1. **Ortam Oluşturma** (`Ortam()`)

```python
app = Ortam()
```

**Yapılanlar:**
- Ursina penceresi oluşturulur
- Su hacmi, deniz tabanı, çimen katmanı oluşturulur
- Havuz genişliği: 200 birim (varsayılan)
- Su yüksekliği: 100 birim
- Deniz tabanı kalınlığı: 10 birim (su yüksekliğinin %10'u)
- Çimen katmanı kalınlığı: 25 birim (su yüksekliğinin %25'i)

---

### 2. **Simülasyon Nesneleri Oluşturma** (`sim_olustur()`)

```python
app.sim_olustur(n_rovs=4, n_engels=25, hedef_nokta=Vec3(40, 0, 60))
```

#### **2.1. Hedef Nokta Belirleme**
- Hedef nokta: `Vec3(40, 0, 60)` (main.py'de tanımlı)
- Engeller bu noktadan **30 birim** uzakta oluşturulur
- ROV'lar bu noktadan **30 birim** uzakta başlatılır

#### **2.2. Engeller (Kayalar) Oluşturma**
- **25 engel** oluşturulur
- **Pozisyon:**
  - X: `-90` ile `+90` arası (havuz genişliğinin %90'ı)
  - Z: `-90` ile `+90` arası
  - Y: `-90` ile `-10` arası (su içinde)
- **Hedeften uzaklık:** Minimum 30 birim
- **Boyut:** 4-12 birim arası rastgele
- **Renk:** 40-200 arası gri tonları (benek efekti ile)
- **Texture:** `noise` (gerçekçi kaya görünümü)

#### **2.3. ROV'lar Oluşturma**
- **4 ROV** oluşturulur
- **Başlangıç Pozisyonu:**
  - X: `-30` ile `+30` arası (havuz genişliğinin %30'ı - merkez alan)
  - Z: `-30` ile `+30` arası
  - Y: `-2` (su yüzeyine yakın)
- **Güvenlik Kontrolleri:**
  - Engellerden minimum **15 birim** uzakta
  - Hedeften minimum **30 birim** uzakta
  - Eğer geçerli pozisyon bulunamazsa: Varsayılan pozisyon `(-20 + i*10, -2, -20 + i*10)`

**ROV Başlangıç Özellikleri:**
```python
- velocity: Vec3(0, 0, 0)  # Sıfır hız
- battery: 100.0  # %100 pil
- role: 0  # Takipçi (henüz lider atanmamış)
- color: color.orange  # Turuncu
- manuel_kontrol: False  # Otomatik mod (henüz GNC kurulmamış)
```

---

### 3. **GNC Sistemi Kurulumu** (`otomatik_kurulum()`)

```python
filo.otomatik_kurulum(
    rovs=app.rovs,
    lider_id=0,
    baslangic_hedefleri={
        0: (40, 0, 60),    # Lider
        1: (35, -10, 50),  # Takipçi 1
        2: (40, -10, 50),  # Takipçi 2
        3: (45, -10, 50)   # Takipçi 3
    }
)
```

#### **3.1. Her ROV İçin İşlemler**

**Lider ROV (ID=0):**
1. **Rol Ata:** `rov.set("rol", 1)` → Lider olur
2. **Renk:** Kırmızı (`color.red`)
3. **Pozisyon:** Su yüzeyine çıkar (`y = 0`)
4. **Modem Oluştur:** Lider modem ayarları ile
5. **GNC Sistemi:** `LiderGNC` oluşturulur
6. **Başlangıç Hedefi:** `(40, 60, 0)` → `filo.git(0, 40, 60, 0)`
   - Manuel kontrol: **KAPALI** (otomatik mod)
   - AI: **AÇIK**
   - Hedef: `Vec3(40, 0, 60)`

**Takipçi ROV'lar (ID=1,2,3):**
1. **Rol Ata:** `rov.set("rol", 0)` → Takipçi olur
2. **Renk:** Turuncu (`color.orange`)
3. **Modem Oluştur:** Takipçi modem ayarları ile
4. **GNC Sistemi:** `TakipciGNC` oluşturulur
5. **Başlangıç Hedefi:**
   - ROV-1: `(35, 50, -10)` → `filo.git(1, 35, 50, -10)`
   - ROV-2: `(40, 50, -10)` → `filo.git(2, 40, 50, -10)`
   - ROV-3: `(45, 50, -10)` → `filo.git(3, 45, 50, -10)`
   - Manuel kontrol: **KAPALI** (otomatik mod)
   - AI: **AÇIK**

#### **3.2. `git()` Fonksiyonu Etkisi**

Her `filo.git()` çağrısı:
- ✅ Manuel kontrolü **KAPATIR** (`manuel_kontrol = False`)
- ✅ AI'yı **AÇAR** (`ai_aktif = True`)
- ✅ Hedef atar (`hedef_atama()`)
- ✅ ROV otomatik olarak hedefe gitmeye başlar

---

### 4. **İlk Frame'de Ne Olur?**

#### **4.1. ROV Update Döngüsü**

Her ROV'un `update()` fonksiyonu çağrılır:

1. **Manuel Hareket Kontrolü:**
   - `manuel_hareket['yon']` kontrol edilir
   - Başlangıçta `None` (manuel hareket yok)

2. **Havuz Sınır Kontrolü:**
   - X ve Z eksenlerinde sınır kontrolü
   - Sınırda hız sıfırlanır

3. **Engel Tespiti:**
   - `_engel_tespiti()` çağrılır
   - En yakın engel bulunur
   - Kesikli çizgi çizilir (eğer menzil içindeyse)

4. **Sonar İletişim:**
   - `_sonar_iletisim()` çağrılır
   - Yakın ROV'lar tespit edilir
   - İletişim çizgileri çizilir

5. **Fizik:**
   - Pozisyon güncellenir: `position += velocity * time.dt`
   - Sürtünme uygulanır: `velocity *= 0.95`

6. **Çarpışma Kontrolü:**
   - ROV-ROV çarpışması
   - ROV-Engel çarpışması

7. **Lider Özel Davranışı:**
   - Su yüzeyinde kalır (`y >= 0`)
   - Batırılamaz (`velocity.y >= 0`)

#### **4.2. GNC Update Döngüsü**

`main.py`'deki `update()` fonksiyonu:

1. **GAT Verisi Al:**
   ```python
   veri = app.simden_veriye()
   ```

2. **GAT Analizi:**
   ```python
   tahminler, _, _ = beyin.analiz_et(veri)
   # Başlangıçta genellikle [0, 0, 0, 0] (OK durumu)
   ```

3. **Renk Güncelleme:**
   - Lider: Kırmızı
   - Takipçiler: GAT koduna göre (başlangıçta turuncu - OK)

4. **GNC Güncelleme:**
   ```python
   filo.guncelle_hepsi(tahminler)
   ```
   - Her ROV'un `guncelle()` fonksiyonu çağrılır
   - Hedefe doğru hareket başlar

---

## 🎯 Başlangıç Davranış Özeti

### **Lider ROV (ROV-0):**
- ✅ **Pozisyon:** Su yüzeyinde (`y = 0`)
- ✅ **Hedef:** `(40, 0, 60)`
- ✅ **Mod:** Otomatik (manuel kontrol kapalı)
- ✅ **AI:** Açık
- ✅ **Renk:** Kırmızı
- ✅ **Davranış:** Hedefe doğru ilerler, su yüzeyinde kalır

### **Takipçi ROV'lar (ROV-1,2,3):**
- ✅ **Pozisyon:** Su yüzeyine yakın (`y = -2`)
- ✅ **Hedefler:**
  - ROV-1: `(35, -10, 50)`
  - ROV-2: `(40, -10, 50)`
  - ROV-3: `(45, -10, 50)`
- ✅ **Mod:** Otomatik (manuel kontrol kapalı)
- ✅ **AI:** Açık
- ✅ **Renk:** Turuncu (OK durumu)
- ✅ **Davranış:** Hedeflerine doğru ilerler, formasyon oluşturur

---

## 🔧 Başlangıç Parametreleri

### **Varsayılan Değerler (Eğer `baslangic_hedefleri` verilmezse):**

**Lider:**
```python
self.git(0, 40, 60, 0)  # (40, 0, 60)
```

**Takipçiler:**
```python
offset_x = 30 + (i * 5)
self.git(i, offset_x, 50, -10)
# ROV-1: (35, -10, 50)
# ROV-2: (40, -10, 50)
# ROV-3: (45, -10, 50)
```

---

## 📊 Başlangıç Durumu Tablosu

| ROV | Pozisyon | Hedef | Rol | Mod | AI | Renk |
|-----|----------|-------|-----|-----|----|----|
| ROV-0 | Su yüzeyi | (40, 0, 60) | Lider | Otomatik | Açık | Kırmızı |
| ROV-1 | Yüzeye yakın | (35, -10, 50) | Takipçi | Otomatik | Açık | Turuncu |
| ROV-2 | Yüzeye yakın | (40, -10, 50) | Takipçi | Otomatik | Açık | Turuncu |
| ROV-3 | Yüzeye yakın | (45, -10, 50) | Takipçi | Otomatik | Açık | Turuncu |

---

## 🎮 İlk Hareket

Simülasyon başladığında:

1. **ROV'lar hedeflerine doğru hareket eder**
2. **Lider su yüzeyinde kalır**
3. **Takipçiler formasyon oluşturur**
4. **GAT analizi yapılır** (her frame)
5. **Engel tespiti çalışır** (her frame)
6. **İletişim çizgileri görünür** (menzil içindeyse)

---

## 💡 Önemli Notlar

1. **Manuel Kontrol:** Başlangıçta **KAPALI** - ROV'lar otomatik hedefe gider
2. **AI:** Başlangıçta **AÇIK** - GAT tahminleri kullanılır
3. **Hız:** Başlangıçta **0** - ROV'lar yavaşça hızlanır (momentum korunumu)
4. **Pil:** Başlangıçta **%100** - Hareket ettikçe azalır
5. **Formasyon:** Takipçiler lideri takip eder (otomatik)

---

## 🔄 Değiştirilebilir Parametreler

### **main.py'de:**
```python
# Hedef nokta
hedef_nokta = Vec3(40, 0, 60)

# ROV ve engel sayısı
app.sim_olustur(n_rovs=4, n_engels=25, hedef_nokta=hedef_nokta)

# Başlangıç hedefleri
baslangic_hedefleri={
    0: (40, 0, 60),    # Lider
    1: (35, -10, 50),  # Takipçi 1
    2: (40, -10, 50),  # Takipçi 2
    3: (45, -10, 50)   # Takipçi 3
}
```

Bu parametreleri değiştirerek başlangıç davranışlarını özelleştirebilirsiniz!

