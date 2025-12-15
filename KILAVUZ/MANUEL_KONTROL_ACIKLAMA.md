# 🎮 Manuel Kontrol Nasıl Açılır?

## 📋 Manuel Kontrol Nedir?

Manuel kontrol modu, ROV'un otomatik navigasyon sistemini (GNC) devre dışı bırakır ve kullanıcının doğrudan ROV'u kontrol etmesine izin verir.

**Manuel Kontrol Açıkken:**
- ✅ ROV otomatik hedefe gitmez
- ✅ GAT tahminleri görmezden gelinir
- ✅ Kullanıcı ROV'u manuel olarak hareket ettirebilir
- ✅ `guncelle()` fonksiyonu çalışmaz (return eder)

**Manuel Kontrol Kapalıyken:**
- ✅ ROV otomatik hedefe gider
- ✅ GAT tahminlerine göre hareket eder
- ✅ AI destekli navigasyon aktif

---

## 🔧 Manuel Kontrolü Açma Yöntemleri

### 1. **`move()` Fonksiyonu ile (Önerilen)**

`move()` fonksiyonunu kullandığınızda manuel kontrol **otomatik olarak açılır** ve ROV güç bazlı hareket eder.

```python
# Manuel kontrolü aç ve ROV'u güç bazlı hareket ettir
filo.move(0, 'ileri', 1.0)   # ROV-0: Manuel kontrol AÇIK, %100 güçle ileri
filo.move(1, 'sag', 0.5)     # ROV-1: Manuel kontrol AÇIK, %50 güçle sağa
filo.move(2, 'cik', 0.3)     # ROV-2: Manuel kontrol AÇIK, %30 güçle yukarı
filo.move(3, 'dur', 0.0)     # ROV-3: Manuel kontrol AÇIK, dur
```

**Özellikler:**
- ✅ Manuel kontrol otomatik açılır
- ✅ Güç bazlı hareket (0.0-1.0 arası, gerçek dünya gibi)
- ✅ Havuz sınır kontrolü otomatik (sınırda hareket engellenir)
- ✅ Lider ROV batırılamaz kontrolü
- ✅ Sürekli hareket (her frame güç uygulanır)
- ✅ Tek komutla hem kontrol açılır hem hareket edilir

---

### 2. **Doğrudan `manuel_kontrol` Özelliğini Ayarlama**

GNC sistemine doğrudan erişerek manuel kontrolü açabilirsiniz.

```python
# Manuel kontrolü aç
filo.sistemler[0].manuel_kontrol = True   # ROV-0 için manuel kontrol AÇIK
filo.sistemler[1].manuel_kontrol = True   # ROV-1 için manuel kontrol AÇIK

# Manuel kontrolü kapat
filo.sistemler[0].manuel_kontrol = False  # ROV-0 için manuel kontrol KAPALI
```

**Kullanım Senaryosu:**
```python
# ROV-0'ı manuel kontrol moduna al
filo.sistemler[0].manuel_kontrol = True

# Şimdi ROV'u manuel olarak hareket ettir
rovs[0].move("ileri", 10)  # Direkt ROV entity'sinden hareket
rovs[0].move("sag", 5)

# Manuel kontrolü kapat ve otomatik moda dön
filo.sistemler[0].manuel_kontrol = False
filo.git(0, 40, 60, 0)  # Otomatik hedefe git
```

---

## 🔄 Manuel Kontrolü Kapatma

### 1. **`git()` Fonksiyonu ile (Önerilen)**

`git()` fonksiyonunu kullandığınızda manuel kontrol **otomatik olarak kapanır** ve otomatik navigasyon başlar.

```python
# Manuel kontrolü kapat ve otomatik hedefe git
filo.git(0, 40, 60, 0)  # ROV-0: Manuel kontrol KAPALI, otomatik hedefe git
filo.git(1, 35, 50, -10)  # ROV-1: Manuel kontrol KAPALI, otomatik hedefe git
```

---

### 2. **Doğrudan `manuel_kontrol` Özelliğini Ayarlama**

```python
# Manuel kontrolü kapat
filo.sistemler[0].manuel_kontrol = False  # ROV-0 için manuel kontrol KAPALI
```

---

## 📊 Manuel Kontrol Durumunu Kontrol Etme

```python
# ROV'un manuel kontrol durumunu kontrol et
durum = filo.sistemler[0].manuel_kontrol
if durum:
    print("ROV-0 manuel kontrol modunda")
else:
    print("ROV-0 otomatik kontrol modunda")
```

---

## 🎯 Örnek Senaryolar

### Senaryo 1: Manuel Hareket Sonra Otomatik Moda Dön

```python
# 1. ROV-0'ı manuel kontrol et
filo.move(0, 'ileri', 5.0)   # Manuel kontrol AÇIK, 5 birim ileri
filo.move(0, 'sag', 3.0)     # Manuel kontrol AÇIK, 3 birim sağa

# 2. Otomatik moda dön
filo.git(0, 50, 70, -10)    # Manuel kontrol KAPALI, otomatik hedefe git
```

---

### Senaryo 2: Belirli ROV'ları Manuel, Diğerlerini Otomatik

```python
# ROV-0 ve ROV-1'i manuel kontrol et
filo.move(0, 'ileri', 10)   # ROV-0: Manuel
filo.move(1, 'sag', 5)      # ROV-1: Manuel

# ROV-2 ve ROV-3 otomatik modda kalsın
# (zaten otomatik modda, bir şey yapmaya gerek yok)
```

---

### Senaryo 3: Acil Durumda Tüm ROV'ları Manuel Kontrole Al

```python
# Tüm ROV'ları manuel kontrol moduna al
for i in range(len(filo.sistemler)):
    filo.sistemler[i].manuel_kontrol = True
    print(f"ROV-{i} manuel kontrol moduna alındı")

# Şimdi tüm ROV'ları manuel olarak hareket ettir
for i in range(len(filo.sistemler)):
    rovs[i].move("cik", 5)  # Tüm ROV'lar yukarı çıksın
```

---

### Senaryo 4: Manuel Kontrol Sonrası Otomatik Formasyon

```python
# 1. ROV'ları manuel olarak konumlandır
filo.move(0, 'ileri', 20)   # ROV-0
filo.move(1, 'sag', 10)     # ROV-1
filo.move(2, 'sol', 10)     # ROV-2
filo.move(3, 'geri', 5)     # ROV-3

# 2. Formasyon hedefi ver
filo.git(0, 50, 60, 0)      # Lider hedefe
filo.git(1, 45, 55, -10)    # Takipçi 1
filo.git(2, 55, 55, -10)    # Takipçi 2
filo.git(3, 50, 50, -10)    # Takipçi 3
# Tüm ROV'lar otomatik moda geçer ve hedeflerine gider
```

---

## ⚠️ Önemli Notlar

1. **Manuel Kontrol Açıkken:**
   - `guncelle_hepsi()` fonksiyonu ROV'u güncellemez
   - GAT tahminleri görmezden gelinir
   - ROV otomatik hedefe gitmez

2. **Manuel Kontrol Kapalıyken:**
   - `git()` ile hedef verildiğinde otomatik navigasyon başlar
   - GAT tahminlerine göre hareket eder
   - AI destekli kaçınma aktif

3. **Havuz Sınırları:**
   - `move()` fonksiyonu havuz sınırlarını otomatik kontrol eder
   - Manuel kontrolde bile sınırlar korunur

4. **Lider ROV:**
   - Lider ROV `move()` ile batırılamaz
   - `bat` komutu lider için çalışmaz

---

## 🔍 Kod İçinde Nasıl Çalışır?

### `move()` Fonksiyonu:
```python
def move(self, rov_id, yon, birim=1.0):
    if 0 <= rov_id < len(self.sistemler):
        # Manuel kontrolü aç
        self.sistemler[rov_id].manuel_kontrol = True
        # ... hareket işlemleri ...
```

### `git()` Fonksiyonu:
```python
def git(self, rov_id, x, z, y=None, ai=True):
    if 0 <= rov_id < len(self.sistemler):
        # Manuel modu kapat, otopilotu aç
        self.sistemler[rov_id].manuel_kontrol = False
        # ... hedef atama ...
```

### `guncelle()` Fonksiyonu (LiderGNC/TakipciGNC):
```python
def guncelle(self, gat_kodu):
    if self.manuel_kontrol: return  # Manuel kontrol açıksa çalışmaz
    # ... otomatik navigasyon ...
```

---

## 📝 Özet

| Yöntem | Manuel Kontrol | Otomatik Kontrol |
|--------|---------------|------------------|
| `move()` | ✅ AÇIK | ❌ KAPALI |
| `git()` | ❌ KAPALI | ✅ AÇIK |
| `manuel_kontrol = True` | ✅ AÇIK | ❌ KAPALI |
| `manuel_kontrol = False` | ❌ KAPALI | ✅ AÇIK |

**En Kolay Yöntem:**
- Manuel kontrol için: `filo.move(rov_id, yon, birim)`
- Otomatik kontrol için: `filo.git(rov_id, x, z, y)`

