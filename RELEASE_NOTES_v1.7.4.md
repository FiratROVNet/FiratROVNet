# Release Notes v1.7.4

## 📅 Tarih
2024

## 🎯 Özet
Bu sürüm, ROV görselleştirme iyileştirmeleri ve senaryo modülü geliştirmeleri içermektedir.

## ✨ Yeni Özellikler

### ROV Görselleştirme İyileştirmeleri
- **ROV Etiketleri Büyütüldü**: ROV üzerindeki label ve GAT bilgileri scale 5'ten 12'ye çıkarıldı
  - Uzaktan bakıldığında ROV bilgileri daha rahat okunabilir
  - `main.py` update döngüsünde label scale kontrolü eklendi

### Senaryo Modülü Geliştirmeleri
- Senaryo modülü için formasyon hesaplaması devre dışı bırakıldı
- ROV başlangıç pozisyonlarının korunması sağlandı
- `baslangic_hedefleri={}` ile formasyon hesaplaması atlanıyor

## 🐛 Hata Düzeltmeleri

### Senaryo Modülü
- ROV pozisyonlarının aynı olması sorunu düzeltildi
- Rol ataması çakışması giderildi
- `get()` metoduna fallback mekanizması eklendi
- Minimal ROV objesi için `move()` metodu eklendi
- Fizik güncellemesi düzeltildi (ROV pozisyonları artık doğru güncelleniyor)

### GNC Sistemi
- Senaryo modülü için formasyon hesaplaması atlanıyor (`baslangic_hedefleri={}` kontrolü)
- ROV başlangıç pozisyonları korunuyor

## 📝 Değişiklikler

### Kod İyileştirmeleri
- Senaryo modülünde rol ataması çakışması giderildi
- `otomatik_kurulum` içinde boş dict kontrolü eklendi
- ROV label scale kontrolü `main.py`'ye eklendi

## 🔧 Teknik Detaylar

### Dosya Değişiklikleri
- `FiratROVNet/simulasyon.py`: ROV label scale artırıldı (5 → 12)
- `FiratROVNet/senaryo.py`: Formasyon hesaplaması devre dışı, pozisyon koruma
- `FiratROVNet/gnc.py`: Boş dict kontrolü eklendi
- `main.py`: Label scale kontrolü eklendi
- `FiratROVNet/__init__.py`: Version 1.7.4

## 📦 Bağımlılıklar
Değişiklik yok.

## 🚀 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.4

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.4
```

### ROV Etiketleri
ROV'lar üzerindeki etiketler artık daha büyük ve uzaktan okunabilir.

### Senaryo Modülü
```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20)

# ROV pozisyonları artık doğru korunuyor
gps = senaryo.get(0, 'gps')
print(f"ROV-0 GPS: {gps}")
```

## 🔄 Geriye Uyumluluk
Tüm değişiklikler geriye uyumludur.

## 📚 İlgili Dokümantasyon
- Senaryo modülü: `KILAVUZ/SENARYO_KULLANIM.md`

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team





# Release Notes v1.7.4

## 📅 Tarih
2024

## 🎯 Özet
Bu sürüm, ROV görselleştirme iyileştirmeleri ve senaryo modülü geliştirmeleri içermektedir.

## ✨ Yeni Özellikler

### ROV Görselleştirme İyileştirmeleri
- **ROV Etiketleri Büyütüldü**: ROV üzerindeki label ve GAT bilgileri scale 5'ten 12'ye çıkarıldı
  - Uzaktan bakıldığında ROV bilgileri daha rahat okunabilir
  - `main.py` update döngüsünde label scale kontrolü eklendi

### Senaryo Modülü Geliştirmeleri
- Senaryo modülü için formasyon hesaplaması devre dışı bırakıldı
- ROV başlangıç pozisyonlarının korunması sağlandı
- `baslangic_hedefleri={}` ile formasyon hesaplaması atlanıyor

## 🐛 Hata Düzeltmeleri

### Senaryo Modülü
- ROV pozisyonlarının aynı olması sorunu düzeltildi
- Rol ataması çakışması giderildi
- `get()` metoduna fallback mekanizması eklendi
- Minimal ROV objesi için `move()` metodu eklendi
- Fizik güncellemesi düzeltildi (ROV pozisyonları artık doğru güncelleniyor)

### GNC Sistemi
- Senaryo modülü için formasyon hesaplaması atlanıyor (`baslangic_hedefleri={}` kontrolü)
- ROV başlangıç pozisyonları korunuyor

## 📝 Değişiklikler

### Kod İyileştirmeleri
- Senaryo modülünde rol ataması çakışması giderildi
- `otomatik_kurulum` içinde boş dict kontrolü eklendi
- ROV label scale kontrolü `main.py`'ye eklendi

## 🔧 Teknik Detaylar

### Dosya Değişiklikleri
- `FiratROVNet/simulasyon.py`: ROV label scale artırıldı (5 → 12)
- `FiratROVNet/senaryo.py`: Formasyon hesaplaması devre dışı, pozisyon koruma
- `FiratROVNet/gnc.py`: Boş dict kontrolü eklendi
- `main.py`: Label scale kontrolü eklendi
- `FiratROVNet/__init__.py`: Version 1.7.4

## 📦 Bağımlılıklar
Değişiklik yok.

## 🚀 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.4

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.4
```

### ROV Etiketleri
ROV'lar üzerindeki etiketler artık daha büyük ve uzaktan okunabilir.

### Senaryo Modülü
```python
from FiratROVNet import senaryo

# Senaryo oluştur
senaryo.uret(n_rovs=4, n_engels=20)

# ROV pozisyonları artık doğru korunuyor
gps = senaryo.get(0, 'gps')
print(f"ROV-0 GPS: {gps}")
```

## 🔄 Geriye Uyumluluk
Tüm değişiklikler geriye uyumludur.

## 📚 İlgili Dokümantasyon
- Senaryo modülü: `KILAVUZ/SENARYO_KULLANIM.md`

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team














