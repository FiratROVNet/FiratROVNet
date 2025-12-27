# 🎉 Release v1.7.3

**Tarih**: 2024  
**Versiyon**: 1.7.3  
**Tip**: PATCH Release (Hata Düzeltmeleri ve İyileştirmeler)

---

## ✨ Yeni Özellikler

### 📡 Sonar Sistemi Güncellemesi
- **Sonar bilgisi artık anlamlı değerler döndürüyor:**
  - `-1`: ROV kopuk, hiçbir iletişim almıyor (iletişim menzili dışında)
  - `0`: İletişim var, sorun yok (engel yok)
  - `1`: Engel tespit edildi
- İletişim kontrolü eklendi (yüzey ve su altı iletişimi)
- Engel tespiti iyileştirildi

---

## 🐛 Hata Düzeltmeleri

### 🔧 Kod Organizasyonu
- `simden_veriye()` fonksiyonu `main.py`'den `Ortam` sınıfına taşındı
- `main.py` sadeleştirildi ve gereksiz import'lar kaldırıldı
- Kod organizasyonu iyileştirildi

### 🔋 Batarya Sistemi
- Batarya değerleri 0-1 arası normalize edildi (1.0 = %100 dolu)
- Batarya tüketim formülü güncellendi
- Başlangıç batarya değeri 1.0 olarak ayarlandı

### 🎯 GAT Kod 1 Davranışı
- GAT kod 1 (ENGEL) davranışı eski haline getirildi
- Basit kaçınma algoritması restore edildi
- Güç çarpanı 0.5'e geri döndürüldü (yavaş hareket)

---

## 📝 Değişiklikler

### Kod Refactoring
- `simden_veriye()` artık `app.simden_veriye()` şeklinde çağrılıyor
- `torch` import'u sadece `simulasyon.py`'de kaldı
- `main.py` daha sade ve okunabilir hale getirildi

### Dokümantasyon
- Release versiyon yönetimi rehberi eklendi (`KILAVUZ/RELEASE_VERSIYON_YONETIMI.md`)

---

## 📦 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.3

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.3
```

---

## 🔄 v1.7.2'den v1.7.3'e Geçiş

### Breaking Changes
- ❌ Yok

### Migration Guide
- `simden_veriye()` artık `app.simden_veriye()` olarak çağrılmalı
- Sonar değerleri artık `-1`, `0`, `1` formatında (eskiden mesafe değeri döndürüyordu)
- Batarya değerleri artık 0-1 arası (eskiden 0-100 arasıydı)

---

## 📊 İstatistikler

- **Toplam Commit**: 12
- **Dosya Değişikliği**: 3 ana dosya
- **Eklenen Satır**: ~150
- **Silinen Satır**: ~50

---

## 🙏 Katkıda Bulunanlar

- Ömer Faruk Çelik

---

## 📚 İlgili Dokümantasyon

- [GAT Kodları Rehberi](KILAVUZ/GAT_KODLARI_RENKLER.md)
- [Release Versiyon Yönetimi](KILAVUZ/RELEASE_VERSIYON_YONETIMI.md)
- [Batarya Sistemi](KILAVUZ/BATARYA_SISTEMI.md)

---

**Not**: Bu release, v1.7.2'den sonraki tüm iyileştirmeleri ve hata düzeltmelerini içermektedir.









# 🎉 Release v1.7.3

**Tarih**: 2024  
**Versiyon**: 1.7.3  
**Tip**: PATCH Release (Hata Düzeltmeleri ve İyileştirmeler)

---

## ✨ Yeni Özellikler

### 📡 Sonar Sistemi Güncellemesi
- **Sonar bilgisi artık anlamlı değerler döndürüyor:**
  - `-1`: ROV kopuk, hiçbir iletişim almıyor (iletişim menzili dışında)
  - `0`: İletişim var, sorun yok (engel yok)
  - `1`: Engel tespit edildi
- İletişim kontrolü eklendi (yüzey ve su altı iletişimi)
- Engel tespiti iyileştirildi

---

## 🐛 Hata Düzeltmeleri

### 🔧 Kod Organizasyonu
- `simden_veriye()` fonksiyonu `main.py`'den `Ortam` sınıfına taşındı
- `main.py` sadeleştirildi ve gereksiz import'lar kaldırıldı
- Kod organizasyonu iyileştirildi

### 🔋 Batarya Sistemi
- Batarya değerleri 0-1 arası normalize edildi (1.0 = %100 dolu)
- Batarya tüketim formülü güncellendi
- Başlangıç batarya değeri 1.0 olarak ayarlandı

### 🎯 GAT Kod 1 Davranışı
- GAT kod 1 (ENGEL) davranışı eski haline getirildi
- Basit kaçınma algoritması restore edildi
- Güç çarpanı 0.5'e geri döndürüldü (yavaş hareket)

---

## 📝 Değişiklikler

### Kod Refactoring
- `simden_veriye()` artık `app.simden_veriye()` şeklinde çağrılıyor
- `torch` import'u sadece `simulasyon.py`'de kaldı
- `main.py` daha sade ve okunabilir hale getirildi

### Dokümantasyon
- Release versiyon yönetimi rehberi eklendi (`KILAVUZ/RELEASE_VERSIYON_YONETIMI.md`)

---

## 📦 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.3

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.3
```

---

## 🔄 v1.7.2'den v1.7.3'e Geçiş

### Breaking Changes
- ❌ Yok

### Migration Guide
- `simden_veriye()` artık `app.simden_veriye()` olarak çağrılmalı
- Sonar değerleri artık `-1`, `0`, `1` formatında (eskiden mesafe değeri döndürüyordu)
- Batarya değerleri artık 0-1 arası (eskiden 0-100 arasıydı)

---

## 📊 İstatistikler

- **Toplam Commit**: 12
- **Dosya Değişikliği**: 3 ana dosya
- **Eklenen Satır**: ~150
- **Silinen Satır**: ~50

---

## 🙏 Katkıda Bulunanlar

- Ömer Faruk Çelik

---

## 📚 İlgili Dokümantasyon

- [GAT Kodları Rehberi](KILAVUZ/GAT_KODLARI_RENKLER.md)
- [Release Versiyon Yönetimi](KILAVUZ/RELEASE_VERSIYON_YONETIMI.md)
- [Batarya Sistemi](KILAVUZ/BATARYA_SISTEMI.md)

---

**Not**: Bu release, v1.7.2'den sonraki tüm iyileştirmeleri ve hata düzeltmelerini içermektedir.


























