# ❌ `IndexError: list index out of range` Hatası Çözümü

## 🔴 Hata

```python
>>> from FiratROVNet.gnc import Filo
>>> filo = Filo()
>>> filo.sistemler[0]
Traceback (most recent call last):
  File "<console>", line 1, in module>
IndexError: list index out of range
```

## 🔍 Hatanın Nedeni

`Filo()` sınıfı oluşturulduğunda `sistemler` listesi **boş** olarak başlatılır:

```python
class Filo:
    def __init__(self):
        self.sistemler = []  # ← Boş liste!
```

ROV'lar ve GNC sistemleri **sadece** `otomatik_kurulum()` fonksiyonu çağrıldığında eklenir.

---

## ✅ Çözüm

### 1. **`otomatik_kurulum()` Fonksiyonunu Çağırın**

ROV'ları eklemek için `otomatik_kurulum()` fonksiyonunu kullanmalısınız:

```python
from FiratROVNet.gnc import Filo
from FiratROVNet.simulasyon import Ortam

# 1. Simülasyon ortamını oluştur
app = Ortam()
app.sim_olustur(n_rovs=4, n_engels=15)

# 2. Filo'yu oluştur
filo = Filo()

# 3. Otomatik kurulum yap (ROV'lar burada eklenir)
tum_modemler = filo.otomatik_kurulum(
    rovs=app.rovs,  # ← ROV listesi gerekli
    lider_id=0
)

# 4. Artık sistemler listesi dolu
print(len(filo.sistemler))  # 4 (4 ROV için 4 GNC sistemi)
print(filo.sistemler[0])    # ✅ Çalışır!
print(filo.sistemler[1])    # ✅ Çalışır!
```

---

### 2. **Manuel Ekleme (Gelişmiş Kullanım)**

Eğer manuel olarak GNC sistemleri eklemek istiyorsanız:

```python
from FiratROVNet.gnc import Filo, LiderGNC, TakipciGNC
from FiratROVNet.iletisim import AkustikModem

filo = Filo()

# Manuel olarak GNC sistemleri ekle
rov = app.rovs[0]
modem = AkustikModem(rov_id=0)
gnc = LiderGNC(rov, modem)
filo.ekle(gnc)  # ← Manuel ekleme

# Artık erişilebilir
print(filo.sistemler[0])  # ✅ Çalışır!
```

---

## 📋 Tam Örnek

### Konsol Üzerinden Kullanım:

```python
# 1. Simülasyonu başlat (main.py çalıştırıldıktan sonra)
# main.py içinde zaten filo oluşturulmuş olmalı

# 2. Konsolda filo'ya eriş
>>> filo.sistemler[0]  # ✅ Çalışır (eğer otomatik_kurulum çağrıldıysa)
```

### Python Script'inden Kullanım:

```python
from FiratROVNet.gnc import Filo
from FiratROVNet.simulasyon import Ortam

# Simülasyon ortamı
app = Ortam()
app.sim_olustur(n_rovs=4, n_engels=15)

# Filo oluştur
filo = Filo()

# ÖNEMLİ: Otomatik kurulum yap
filo.otomatik_kurulum(rovs=app.rovs, lider_id=0)

# Artık erişilebilir
print(f"ROV sayısı: {len(filo.sistemler)}")
for i, gnc in enumerate(filo.sistemler):
    print(f"ROV-{i}: {type(gnc).__name__}")
```

---

## ⚠️ Önemli Notlar

1. **`otomatik_kurulum()` Zorunlu:**
   - ROV'ları eklemek için `otomatik_kurulum()` **mutlaka** çağrılmalıdır
   - Bu fonksiyon olmadan `sistemler` listesi boş kalır

2. **ROV Listesi Gerekli:**
   - `otomatik_kurulum()` için `rovs` parametresi (ROV listesi) gerekir
   - ROV listesi `Ortam.sim_olustur()` ile oluşturulur

3. **main.py'de Otomatik:**
   - `main.py` dosyasında `otomatik_kurulum()` zaten çağrılıyor
   - Konsolda `filo` değişkenine erişirken hata alıyorsanız, `main.py`'nin çalıştığından emin olun

---

## 🔧 Hata Kontrolü

Eğer hala hata alıyorsanız, kontrol edin:

```python
# 1. Sistemler listesinin dolu olup olmadığını kontrol et
print(f"Sistem sayısı: {len(filo.sistemler)}")

# 2. Eğer boşsa, otomatik kurulum yap
if len(filo.sistemler) == 0:
    print("⚠️ Sistemler listesi boş! otomatik_kurulum() çağrılmalı.")
    # Otomatik kurulum yap
    filo.otomatik_kurulum(rovs=app.rovs, lider_id=0)

# 3. Artık erişilebilir
if len(filo.sistemler) > 0:
    print(f"✅ İlk ROV: {filo.sistemler[0]}")
```

---

## 📝 Özet

| Durum | `sistemler` Listesi | Erişim |
|-------|-------------------|--------|
| `Filo()` oluşturuldu | `[]` (boş) | ❌ `IndexError` |
| `otomatik_kurulum()` çağrıldı | `[GNC1, GNC2, ...]` (dolu) | ✅ Çalışır |

**Çözüm:** `otomatik_kurulum(rovs=app.rovs)` çağrılmalıdır!

