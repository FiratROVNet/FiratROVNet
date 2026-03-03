# 🔹 Formasyon Seçim - Async/Worker Pattern Kılavuz

> **Problem Çözmek**: `_formasyon_sec` methodu thread'te çağrıldığında return değerleri konsolda görünmüyordu (sadece `<Future>` adresi gösteriliyordu).

> **Çözüm**: Result **cache sistemi** + **console helper methods** 

---

## 🎯 Yeni Sistem Mimarisi

```
┌─────────────────────────────────────────────────┐
│ Filo.formasyon_sec(*args)                       │  ← Konsoldan çağrı
│ ↓ (ThreadPoolExecutor ile çalışır)             │
│ helper._formasyon_sec_impl()                    │  ← Background thread'de çalışır
│ ├─ Formasyon seçimi hesaplanır                 │
│ ├─ self.cache_formasyon_result(sonuc) ← !!    │  ← SONUÇ CACHE'E YAZILIR
│ └─ 'f_id', 'aralik', 'merkez', 'yaw' döner    │
│                                                 │
│ 📦 HTML'de Cache tutulur:                       │
│    - self.last_formasyon_result (dict/None)   │
│    - self.formasyon_result_timestamp (time)   │
│    - self.formasyon_future (Future object)    │
│                                                 │
│ Konsol (Main thread'den):                       │
│ ├─ filo.formasyon_durumu()  → {runnning, done}│
│ ├─ filo.formasyon_sonucu()  → Cache'deki sonuç│
│ └─ filo.formasyon_bekle()   → Sonunu bekle    │
└─────────────────────────────────────────────────┘
```

---

## 💡 Kullanım Örnekleri

### **Senaryo 1: Sonuç Bekleme (Blocking)**

```python
# Konsoldan:
filo.formasyon_sec(dinamik=True)                # Async çağrı → Future döner
sonuc = filo.formasyon_bekle(timeout=5)         # Sonunu bekleme ve al
print(sonuc)
# Çıktı: {'f_id': 2, 'aralik': 25.5, 'merkez': (100.0, 50.0), 'yaw': 90}
```

### **Senaryo 2: Non-blocking (Polling)**

```python
# Konsoldan:
filo.formasyon_sec(dinamik=True)                # Başlat
# ... diğer işler yap ...
time.sleep(0.5)                                 # Thread'in işlemesi için bekle
durumu = filo.formasyon_durumu()                # Kontrol et
print(durumu)
# Çıktı: {'running': False, 'done': True, 'sonuc': {'f_id': 2, ...}, 'exception': None}

sonuc = filo.formasyon_sonucu()                 # Cache'den oku
print(sonuc)
# Çıktı: {'sonuc': {'f_id': 2, ...}, 'zaman': <timestamp>}
```

### **Senaryo 3: Sonucu Kullan ve Temizle**

```python
# Konsoldan:
filo.formasyon_sec(g_id=0)
time.sleep(0.2)
result_dict = filo.formasyon_sonucu(clear=False)    # Oku ama temizleme
print(result_dict['sonuc'])

# ... başka işler ...

result_dict = filo.formasyon_sonucu(clear=True)     # Oku ve temizle
print(result_dict)  # Sonuç hala var, ama cache temizlendi
```

---

## 🔧 Backend Implementasyon

### **1. Cache Sistemi (FiloHelper - core.py)**

```python
class FiloHelper:
    def __init__(self, filo_ref):
        # ... diğer init ...
        
        # 🔹 Worker results cache
        self.last_formasyon_result = None          # Son sonuç
        self.formasyon_result_timestamp = None     # Zamanı
        self.formasyon_future = None               # Active Future tracking
```

### **2. Sonucu Cache'e Yazma (formation.py)**

```python
def _formasyon_sec_impl(self, ...):
    try:
        # ... formasyon seçim mantığı ...
        
        if best_overall:
            result = {
                'f_id': int(b['f_id']),
                'aralik': round(float(b['aralik']), 1),
                'merkez': (round(b['merkez'][0], 2), round(b['merkez'][1], 2)),
                'yaw': float(b['yaw'])
            }
            self.cache_formasyon_result(result)  # 🔹 CACHE'E YAZ
            return result
        
        self.cache_formasyon_result(None)  # Başarısız durumda da cache'e yaz
        return None
    except Exception as e:
        self.cache_formasyon_result(None)  # Hata durumunda da cache'e yaz
        return None
```

### **3. Cache Getter (formation.py)**

```python
def cache_formasyon_result(self, result):
    """Sonucu cache'e kaydet"""
    self.last_formasyon_result = result
    self.formasyon_result_timestamp = time.time()
    if result:
        print(f"📦 [CACHE] Formasyon sonucu saklandı: {result}")

def get_formasyon_result(self, clear=False):
    """Cache'den sonucu al"""
    result = {
        'sonuc': self.last_formasyon_result,
        'zaman': self.formasyon_result_timestamp
    }
    if clear:
        self.last_formasyon_result = None
        self.formasyon_result_timestamp = None
    return result
```

### **4. Console Wrappers (gnc/__init__.py - Filo sınıfı)**

```python
def formasyon_sonucu(self, clear=False):
    """Cache'deki sonucu getir"""
    return self.helper.get_formasyon_result(clear=clear)

def formasyon_durumu(self):
    """Future'ın durumunu kontrol et"""
    future = self.helper.formasyon_future
    ret = {'running': future.running(), 'done': future.done(), 'sonuc': None}
    if future.done():
        ret['sonuc'] = future.result()
    return ret

def formasyon_bekle(self, timeout=5):
    """Future'ın bitmesini bekle"""
    return self.helper.formasyon_future.result(timeout=timeout)
```

---

## 🎬 Tam İş Akışı (Console'dan)

```python
# ✓ 1. Formasyon seçimini başlat (background thread'de çalışacak)
future = filo.formasyon_sec(margin=25, dinamik=True, g_id=0)
print(f"Future object: {future}")  # <Future at 0x...> şeklinde yazılacak

# ✓ 2. Thread'in işlemesi için bekle (kısa bir sure)
import time
time.sleep(0.1)

# ✓ 3. Durumu kontrol et
status = filo.formasyon_durumu()
print(status)
# {'running': False, 'done': True, 'sonuc': {'f_id': 2, 'aralik': 25.0, ...}, 'exception': None}

# ✓ 4. Cache'den sonucu al
sonuc = filo.formasyon_sonucu()
print(sonuc)
# {'sonuc': {'f_id': 2, 'aralik': 25.0, 'merkez': (100, 50), 'yaw': 90}, 'zaman': 1708300500.123}

# ✓ 5. Future'dan direkt al (async-native)
result = filo.formasyon_bekle(timeout=5)
print(result)
# {'f_id': 2, 'aralik': 25.0, 'merkez': (100, 50), 'yaw': 90}
```

---

## 🐛 Sorun Giderme

### **Q: Cache boş mu? Sonuç görünmüyor?**
```python
# A: Thread henüz tamamlanmamış ve cache'e yazılmamış
filo.formasyon_durumu()  # 'running': True çıktısı
time.sleep(1)             # Daha bekle
filo.formasyon_sonucu()   # Tekrar dene
```

### **Q: Timeout hatası alıyorum?**
```python
# A: Formasyon seçim alg. uzun sürüyor veya hung thread
filo.formasyon_durumu()  # 'running': True mi kontrol et
# Eğer async varsa, app.destroy() veya timeout artır
```

### **Q: Multiple formasyon_sec() çağrıları?**
```python
# A: Son Future track ediliyor. Eski Future'ını result() almadan önceki çağırılırsa:
f1 = filo.formasyon_sec()  # Future-1
f2 = filo.formasyon_sec()  # Future-2 (helper.formasyon_future = f2)
# f1 track edilmemiyor! Sonucu almanız gereğiyor:
r1 = f1.result()           # Future-1 sonucu (cache'de değil!)
```

---

## 📊 Benchmark

| Yöntem | Engelleyen | Avantaj |
|--------|-----------|---------|
| `future.result()` | ✓ Evet | Tam kontrol, basit |
| `formasyon_bekle()` | ✓ Evet | Timeout, error handling |
| `formasyon_sonucu()` | ✗ Hayır | Hızlı, non-blocking |
| `formasyon_durumu()` | ✗ Hayır | Detaylı info, real-time |

---

## 🔗 İlgili Dosyalar

- **core.py**: Cache initialization
- **formation.py** (mixin): `_formasyon_sec_impl`, `cache_formasyon_result`, `get_formasyon_result`  
- **gnc/__init__.py**: `formasyon_sonucu()`, `formasyon_durumu()`, `formasyon_bekle()`

---

## 🎓 Best Practices

1. **Her async çağrıdan sonra wait:**
   ```python
   filo.formasyon_sec(...)
   time.sleep(0.1)  # Thread hazırlanması için
   ```

2. **Batch işlemler:**
   ```python
   for group_id in range(3):
       filo.formasyon_sec(g_id=group_id)
   time.sleep(0.5)  # Tümü bekle
   for group_id in range(3):
       print(filo.formasyon_sonucu())  # Tümü oku
   ```

3. **Error checking:**
   ```python
   res = filo.formasyon_sonucu()
   if res['sonuc'] is None:
       print(f"Hata var!")
   ```

---

**Son güncelleme**: February 19, 2026
