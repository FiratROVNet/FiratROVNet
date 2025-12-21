# Release Notes v1.7.5

## 📅 Tarih
2024

## 🎯 Özet
Bu sürüm, 3D ortam görselleştirmesi, ada modeli entegrasyonu, gelişmiş engel algılama sistemi ve kaya oluşum mekanizması iyileştirmeleri içermektedir.

## ✨ Yeni Özellikler

### 3D Ortam Geliştirmeleri
- **Okyanus Tabanı Modeli**: FBX formatında özel okyanus tabanı modeli entegrasyonu
  - `sand_envi_034.fbx` modeli ve texture desteği
  - Deniz tabanına yerleştirilmiş gerçekçi görünüm
  - Fallback mekanizması ile model yoksa sessizce atlanıyor

- **Ada Modeli Entegrasyonu**: Low-poly ada modeli simülasyona eklendi
  - `island1_design2_c4d.obj` modeli ve texture desteği
  - Su yüzeyinin üstünde konumlandırılmış
  - Opaque rendering ile ROV'lar tarafından engel olarak algılanıyor

### Gelişmiş Engel Algılama Sistemi
- **Multi-Layered Hitbox Sistemi**: Ada için çok katmanlı hitbox sistemi
  - 4 farklı derinlik seviyesinde icosphere hitbox'ları
  - Ters koni şeklindeki ada geometrisini doğru şekilde temsil ediyor
  - ROV'lar farklı derinliklerde adanın farklı bölgelerini algılayabiliyor

- **Engel Algılama İyileştirmeleri**:
  - Yatay (X-Z) ve dikey (Y) mesafe hesaplamaları ayrı ayrı yapılıyor
  - Düzleştirilmiş (ellipsoid/pancake) hitbox'lar için doğru algılama
  - Çizgi görselleştirmesi artık engelin yüzeyine çiziliyor (merkezine değil)
  - `dikey_tolerans = 10.0` ile dikey yakınlık kontrolü

### Sensör Menzil Artışları
- **Lider ROV**: Engel algılama menzili 30m → 40m
- **Takipçi ROV**: Engel algılama menzili 10m → 30m
- **Varsayılan**: Engel algılama menzili 10m → 30m
- **Kaçınma Mesafesi**: 5m → 10m (varsayılan)

### Kaya Oluşum Mekanizması
- **Su Altı Kaya Yerleşimi**: Kayalar artık her zaman su altında oluşuyor
  - Tabanları deniz tabanına değiyor
  - Su yüzeyinde yüzen kaya oluşmuyor
  - Kaya boyutuna göre dinamik pozisyon hesaplama
  - Büyük kayalar için özel yerleştirme mantığı

## 🐛 Hata Düzeltmeleri

### Engel Algılama
- ROV'ların düzleştirilmiş hitbox'ları algılayamama sorunu düzeltildi
- Çizgi görselleştirmesinde merkeze çizme hatası giderildi (artık yüzeye çiziliyor)
- Ada hitbox'larının algılanmaması sorunu çözüldü

### Kaya Oluşumu
- Negatif scale değeri hatası düzeltildi (`s_z` artık pozitif)
- Kayaların su yüzeyinde oluşması engellendi
- Kaya pozisyon hesaplaması iyileştirildi

## 📝 Değişiklikler

### Kod İyileştirmeleri
- `_engel_tespiti` fonksiyonu tamamen yeniden yazıldı
  - Yatay ve dikey mesafe hesaplamaları ayrıldı
  - En yakın yüzey noktası hesaplama eklendi
  - Güvenli attribute kontrolü eklendi

- `_kesikli_cizgi_ciz` fonksiyonu güncellendi
  - Artık `hedef_nokta` (Vec3) parametresi alıyor
  - Engel nesnesi yerine hesaplanan yüzey noktasına çiziyor

- Kaya oluşum mantığı yeniden düzenlendi
  - Dinamik alt/üst sınır hesaplama
  - Deniz tabanı ve su yüzeyi referansları kullanılıyor

### Ortam Sınıfı Güncellemeleri
- Ada hitbox'ları `island_hitboxes` listesinde saklanıyor
- `sim_olustur` fonksiyonunda ada hitbox'ları korunuyor
- Deniz tabanı kalınlığı artırıldı (0.1 → 0.15)
- Çimen katmanı kalınlığı artırıldı (0.25 → 0.3)

## 🔧 Teknik Detaylar

### Dosya Değişiklikleri
- `FiratROVNet/simulasyon.py`: 
  - Ada modeli entegrasyonu
  - Multi-layered hitbox sistemi
  - Engel algılama algoritması yeniden yazıldı
  - Kaya oluşum mekanizması iyileştirildi
  - Okyanus tabanı modeli entegrasyonu

- `FiratROVNet/gnc.py`: 
  - Sensör menzil değerleri artırıldı
  - Kaçınma mesafesi artırıldı

- `FiratROVNet/__init__.py`: Version 1.7.5

### Yeni Model Dosyaları
- `Models-3D/lowpoly-island/`: Ada modeli ve texture'ları
- `Models-3D/water/my_models/ocean_taban/`: Okyanus tabanı modeli

## 📦 Bağımlılıklar
Değişiklik yok.

## 🚀 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.5

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.5
```

### Ada Modeli
Ada modeli otomatik olarak yüklenir. Model dosyası yoksa simülasyon normal şekilde devam eder.

### Engel Algılama
ROV'lar artık daha geniş menzilde engelleri algılayabilir:
- Lider ROV: 40m menzil
- Takipçi ROV: 30m menzil

### Kaya Oluşumu
Kayalar artık her zaman su altında ve deniz tabanına değecek şekilde oluşur.

## 🔄 Geriye Uyumluluk
Tüm değişiklikler geriye uyumludur. Model dosyaları yoksa fallback mekanizmaları devreye girer.

## 📚 İlgili Dokümantasyon
- Senaryo modülü: `KILAVUZ/SENARYO_KULLANIM.md`
- Engel algılama: `FiratROVNet/simulasyon.py` içindeki `_engel_tespiti` fonksiyonu

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team




