# 💻 Konsol Erişimi Sorunu Çözümü

## 🔴 Sorun

`main.py` çalıştırdıktan sonra konsola yazamıyorsunuz:

```python
>>> from FiratROVNet.gnc import Filo
>>> filo = Filo()
>>> filo.sistemler[0]
IndexError: list index out of range
```

## 🔍 Sorunun Nedeni

1. **Konsol Thread Olarak Çalışıyor:**
   - `app.run(interaktif=True)` çağrıldığında konsol bir thread olarak başlatılıyor
   - Ursina penceresi açıkken konsol terminalde görünmeyebilir
   - Konsol başlatılması 1 saniye bekliyor (`time.sleep(1)`)

2. **`filo` Değişkeni Konsola Eklenmemişti:**
   - `main.py`'de sadece `gnc` (filo.sistemler) eklenmişti
   - `filo` nesnesi konsola eklenmemişti
   - ✅ **Düzeltildi:** `app.konsola_ekle("filo", filo)` eklendi

---

## ✅ Çözüm

### 1. **Konsol Nasıl Çalışır?**

`main.py` çalıştığında:
1. Ursina penceresi açılır
2. 1 saniye sonra konsol thread'i başlatılır
3. Terminalde şu mesaj görünür:
   ```
   ============================================================
   🚀 FIRAT ROVNET CANLI KONSOL
   Çıkmak için Ctrl+D veya 'exit()' yazın.
   ============================================================
   ```
4. Artık konsola yazabilirsiniz!

### 2. **Konsolda Erişilebilir Değişkenler**

Konsola şu değişkenler otomatik eklenir:

```python
# Otomatik eklenenler
rovs          # ROV listesi
engeller      # Engel listesi
app           # Ortam nesnesi
ursina        # Ursina modülü
cfg           # Config nesnesi

# konsola_ekle() ile eklenenler
git           # filo.git fonksiyonu
gnc           # filo.sistemler (GNC sistemleri listesi)
filo          # Filo nesnesi (✅ YENİ EKLENDİ)
rovs          # ROV listesi
cfg           # Config nesnesi
```

### 3. **Konsol Kullanımı**

#### **Yöntem 1: Konsol Thread'i Bekle**

`main.py` çalıştırdıktan sonra:
1. Ursina penceresi açılır
2. **Terminalde 1 saniye bekleyin**
3. Konsol mesajı görünür:
   ```
   ============================================================
   🚀 FIRAT ROVNET CANLI KONSOL
   Çıkmak için Ctrl+D veya 'exit()' yazın.
   ============================================================
   ```
4. Artık konsola yazabilirsiniz:
   ```python
   >>> filo.sistemler[0]  # ✅ Çalışır!
   >>> filo.git(0, 50, 60, 0)  # ✅ Çalışır!
   >>> rovs[0].move("ileri", 10)  # ✅ Çalışır!
   ```

#### **Yöntem 2: Konsol Mesajını Kontrol Et**

Eğer konsol mesajı görünmüyorsa:
1. Terminali kontrol edin (Ursina penceresi arkasında olabilir)
2. Konsol mesajını bekleyin
3. Eğer görünmüyorsa, konsol thread'i başlatılmamış olabilir

#### **Yöntem 3: Manuel Konsol Başlatma (Gelişmiş)**

Eğer konsol otomatik başlamazsa, manuel olarak başlatabilirsiniz:

```python
# Konsolda (eğer erişebiliyorsanız)
>>> import code
>>> code.interact(local=dict(globals(), **app.konsol_verileri))
```

---

## 🎯 Örnek Kullanım

### Konsolda `filo` Kullanımı:

```python
# Konsol açıldıktan sonra:

# 1. Filo nesnesine eriş
>>> filo.sistemler[0]  # ✅ Çalışır!
>>> filo.sistemler[1]  # ✅ Çalışır!

# 2. ROV'lara hedef ver
>>> filo.git(0, 50, 60, 0)  # ROV-0 hedefe git
>>> filo.git(1, 45, 55, -10)  # ROV-1 hedefe git

# 3. Manuel hareket
>>> filo.move(0, 'ileri', 5.0)  # ROV-0 manuel ileri

# 4. ROV ayarları
>>> filo.set(0, 'rol', 1)  # ROV-0'ı lider yap
>>> filo.get(0, 'gps')  # ROV-0 pozisyonu

# 5. Direkt ROV erişimi
>>> rovs[0].move("ileri", 10)  # ROV-0 ileri
>>> rovs[0].color = color.green  # ROV-0 yeşil
```

---

## ⚠️ Sorun Giderme

### Sorun 1: Konsol Mesajı Görünmüyor

**Çözüm:**
- Terminali kontrol edin (Ursina penceresi arkasında olabilir)
- Konsol thread'i başlatılması için 1-2 saniye bekleyin
- Eğer hala görünmüyorsa, `interaktif=True` parametresini kontrol edin

### Sorun 2: `filo` Değişkeni Bulunamıyor

**Çözüm:**
- ✅ **Düzeltildi:** `app.konsola_ekle("filo", filo)` eklendi
- `main.py`'yi yeniden çalıştırın
- Konsolda `filo` değişkeni artık erişilebilir

### Sorun 3: Konsol Thread'i Başlamıyor

**Çözüm:**
- `simulasyon.py`'de `_start_shell()` fonksiyonunu kontrol edin
- `app.run(interaktif=True)` çağrıldığından emin olun
- Terminal çıktısını kontrol edin

### Sorun 4: Konsol Donuyor

**Çözüm:**
- Ursina penceresi açıkken konsol çalışır
- Konsolu kapatmak için `Ctrl+D` veya `exit()` yazın
- Ursina penceresini kapatmak için `ESC` veya `Q` tuşuna basın

---

## 📝 Özet

| Durum | Konsol Erişimi | Çözüm |
|-------|---------------|-------|
| Konsol mesajı görünüyor | ✅ Erişilebilir | `filo.sistemler[0]` kullan |
| Konsol mesajı görünmüyor | ❌ Erişilemez | 1-2 saniye bekleyin |
| `filo` bulunamıyor | ❌ Hata | ✅ Düzeltildi: `app.konsola_ekle("filo", filo)` eklendi |

**Önemli:**
- Konsol thread'i başlatılması için 1 saniye bekleyin
- Konsol mesajını terminalde kontrol edin
- `filo` değişkeni artık konsola eklenmiş durumda ✅

---

## 🚀 Formasyon Seçim - Async/Worker Pattern

> **Yeni Özellik (v1.7.7+):** Formasyon seçimi background thread'de çalışır!

### 🔹 Problem Çözmü

Thread'de çalışan fonksiyonlar `Future` object döndürür, bu da konsola yazıldığında sadece bellek adresi gösterir:

```python
>>> filo.formasyon_sec()
<Future at 0x7f8c4a5...>  # ❌ Sonuç görünmüyor!
```

### ✅ Çözüm: Helper Methods

3 yeni helper method eklendi:

#### **1. `formasyon_bekle(timeout=5)` - Blocking Wait**

```python
# Sonuç bekleme (blocking - thread tamamlanana kadar engel)
>>> filo.formasyon_sec(dinamik=True)  # Başlat
>>> sonuc = filo.formasyon_bekle(timeout=5)  # Bekleme
✅ Sonuç alındı: {'f_id': 2, 'aralik': 25.0, 'merkez': (100, 50), 'yaw': 90}

# Timeout durumu
>>> sonuc = filo.formasyon_bekle(timeout=1)
⏰ Timeout! 1s içinde sonuç alamadı.
```

#### **2. `formasyon_sonucu(clear=False)` - Non-blocking Read**

```python
# Sonuç okuma (non-blocking - hızlı cache erişimi)
>>> filo.formasyon_sec()  # Başlat
>>> import time
>>> time.sleep(0.2)  # Thread bitiş untuk bekle
>>> sonuc_data = filo.formasyon_sonucu()
>>>print(sonuc_data)
{
    'sonuc': {'f_id': 2, 'aralik': 25.0, 'merkez': (100, 50), 'yaw': 90},
    'zaman': 1708300500.123
}

# Temizle
>>> filo.formasyon_sonucu(clear=True)  # Oku ve cache'i temizle
```

#### **3. `formasyon_durumu()` - Status Check**

```python
# Thread duruma kontrol et (detailed info)
>>> filo.formasyon_sec()  # Başlat
>>> filo.formasyon_durumu()
{
    'running': True,
    'done': False,
    'sonuc': None,
    'exception': None
}

# Tamamlandıktan sonra
>>> time.sleep(0.2)
>>> filo.formasyon_durumu()
{
    'running': False,
    'done': True,
    'sonuc': {'f_id': 2, ...},
    'exception': None
}
```

### 🎯 Kullanım Senaryoları

#### **Senaryo A: Basit Kullanım (Blocking)**

```python
# Sonucu bekleme - en kolay yol
>>> result = filo.formasyon_bekle(timeout=5)
>>> if result:
...     print(f"Formasyon: {result['f_id']}, Aralık: {result['aralik']}")
```

#### **Senaryo B: Batch İşlemler (Non-blocking)**

```python
# Birden fazla grup için aynı anda seçim
>>> for g_id in range(3):
...     filo.formasyon_sec(g_id=g_id, dinamik=True)
>>> time.sleep(1)  # Tümü için yeterli
>>> for g_id in range(3):
...     result = filo.formasyon_sonucu()
...     print(f"Grup-{g_id}: {result['sonuc']}")
```

#### **Senaryo C: Duruma Bağlı İşlem (Polling)**

```python
# Status kontrol et ve duruma göre işlem yap
>>> filo.formasyon_sec()
>>> for i in range(10):
...     status = filo.formasyon_durumu()
...     if status['done']:
...         print(f"✅ Sonuç: {status['sonuc']}")
...         break
...     elif status['running']:
...         print(f"Çalışıyor ({i})...")
...         time.sleep(0.1)
...     time.sleep(0.1)
```

### 📚 Referans Dokumentasyon

Detaylı teknik dokümantasyon:
- **→ [FORMASYON_ASYNC_GUIDE.md](../FORMASYON_ASYNC_GUIDE.md)**

### 🔗 İlgili Konsol Komutları

```python
# Formasyon seçimi
>>> filo.formasyon_sec(dinamik=True, margin=25, g_id=0)

# Formation uygulaması (seçimden sonra)
>>> filo.formasyon(rov_id, formasyon_id, aralık)

# Manual formasyon yapılandırması
>>> filo.formasyon_olustur(rov_idleri=[], formasyon_id=0, aralık=20)
```

### 💡 Best Practices

1. **Timeout değeri seçimi:**
   ```python
   # Hızlı sistem: 2-3 saniye
   result = filo.formasyon_bekle(timeout=3)
   
   # Yavaş sistem veya kompleks geometry: 5-10 saniye
   result = filo.formasyon_bekle(timeout=10)
   ```

2. **Error handling:**
   ```python
   try:
       result = filo.formasyon_bekle(timeout=5)
       if result is None:
           print("Formasyon seçilemedi!")
       else:
           print(f"✅ {result['f_id']} formasyon: {result['aralik']}m")
   except TimeoutError:
       print("⏰ Timeout!")
   ```

3. **Batch işlemlerde coro kullan:**
   ```python
   # İyisi: Non-blocking → batch → oku
   for g_id in range(5):
       filo.formasyon_sec(g_id=g_id)
   time.sleep(1)
   for g_id in range(5):
       print(filo.formasyon_sonucu())
   
   # Kötüsü: Blocking → 1 tane → sıra bekle
   for g_id in range(5):
       result = filo.formasyon_bekle()  # Çok yavaş!
   ```

