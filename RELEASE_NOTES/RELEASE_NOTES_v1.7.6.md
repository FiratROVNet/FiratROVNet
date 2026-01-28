# Release Notes v1.7.6

## 📅 Tarih
2024

## 🎯 Özet
Bu sürüm, formasyon sisteminde liderin yaw açısına göre dinamik rotasyon, hiyerarşik formasyon arama algoritması, dokümantasyon sadeleştirmesi ve GitHub workflow iyileştirmeleri içermektedir.

## ✨ Yeni Özellikler

### Formasyon Sistemi Geliştirmeleri
- **Yaw Açısına Göre Dinamik Rotasyon**: Formasyon pozisyonları artık liderin yaw açısına göre dinamik olarak döndürülüyor
  - 2D rotasyon matrisi ile yaw açısına göre döndürme
  - Lider döndüğünde takipçiler liderin yönüne göre konumlanıyor
  - Formasyon artık global haritaya sabitlenmiş değil, liderin baktığı yöne göre şekilleniyor
  - Tüm formasyon tipleri (LINE, V_SHAPE, DIAMOND, vb.) yaw rotasyonunu destekliyor

- **Hiyerarşik Formasyon Arama Algoritması**: Daha zeki formasyon seçim sistemi
  - **Adım A (Lider Odaklı)**: Önce Lider ROV'un GPS koordinatını merkez kabul eder
  - **Adım B (Dinamik Yaw)**: Mevcut açıyla sığmıyorsa, liderin yaw açısını 0°, 90°, 180°, 270° döndürerek tekrar dener
  - **Adım C (Merkez Odaklı)**: Liderin olduğu yerde hiçbir açıda uygun formasyon bulunamazsa, Hull Merkezi koordinatına geçer
  - Formasyon bulunduğunda liderin yaw açısı otomatik set edilir
  - Hull merkezinde formasyon bulunduysa lider oraya gönderilir

### Dokümantasyon İyileştirmeleri
- **Ana Kullanım Kılavuzu**: Tüm kılavuzlar sadeleştirildi ve tek bir main kılavuz oluşturuldu
  - `KILAVUZ/KULLANIM_KILAVUZU.md`: Kapsamlı kullanım kılavuzu
  - Fonksiyonların kullanımı basit ve anlaşılır şekilde açıklandı
  - Örnek kullanım senaryoları eklendi
  - Hata çözümü bölümü eklendi

## 🐛 Hata Düzeltmeleri

### GitHub Workflow
- `scipy.spatial.ConvexHull bulunamadı` hatası düzeltildi
  - `scipy>=1.9.0` requirements.txt'e eklendi
- `cannot import name 'LiderGNC'` hatası düzeltildi
  - Artık kullanılmayan `LiderGNC` ve `TakipciGNC` importları kaldırıldı
  - `run_tests.py` güncellendi, artık sadece `TemelGNC` kullanılıyor

## 📝 Değişiklikler

### Kod İyileştirmeleri
- `Formasyon.pozisyonlar()` fonksiyonu yaw parametresi ile güncellendi
  - Yerel koordinat sistemi kullanılıyor (lider merkezli)
  - 2D rotasyon matrisi ile yaw açısına göre döndürme
  - Global koordinatlara dönüşüm

- `Filo._formasyon_sec_impl()` fonksiyonu hiyerarşik arama algoritması ile yeniden yazıldı
  - Nokta döngüsü (Lider GPS → Hull Merkezi)
  - Yaw döngüsü (0°, 90°, 180°, 270°)
  - Formasyon tipi döngüsü
  - Aralık döngüsü

### Dokümantasyon Sadeleştirmesi
- Gereksiz kılavuzlar kaldırıldı:
  - `BASLANGIC_DAVRANISLARI.md`
  - `FILO_HATA_COZUMU.md`
  - `GAT_KODLARI_RENKLER.md`
  - `GUNCelle_HEPSI_ACIKLAMA.md`
  - `MANUEL_KONTROL_ACIKLAMA.md`
  - `KOPMA_DAVRANISI.md`
  - `LIST_COMPREHENSION_ATAMA.md`
  - `BATARYA_SISTEMI.md`
  - `3D_MODEL_KULLANIMI.md`
  - `GIT_FLOW_REHBERI.md`
  - `GUVENLI_PUSH_REHBERI.md`
  - `PR_WORKFLOW_REHBERI.md`
  - `RELEASE_VERSIYON_YONETIMI.md`
  - `FILO_KULLANIM.md`

- Yeni ana kılavuz oluşturuldu:
  - `KILAVUZ/KULLANIM_KILAVUZU.md`: Kapsamlı kullanım kılavuzu

## 🔧 Teknik Detaylar

### Dosya Değişiklikleri
- `FiratROVNet/config.py`: 
  - `Formasyon.pozisyonlar()` yaw parametresi eklendi
  - 2D rotasyon matrisi implementasyonu
  - Tüm formasyon tipleri yaw rotasyonunu destekliyor

- `FiratROVNet/gnc.py`: 
  - `Filo._formasyon_sec_impl()` hiyerarşik arama algoritması ile yeniden yazıldı
  - Lider GPS öncelikli kullanım
  - Dinamik yaw açısı denemeleri
  - Hull merkezi fallback mekanizması

- `requirements.txt`: 
  - `scipy>=1.9.0` eklendi (ConvexHull hesaplamaları için)

- `run_tests.py`: 
  - `LiderGNC` ve `TakipciGNC` importları kaldırıldı
  - Artık sadece `TemelGNC` kullanılıyor

- `FiratROVNet/__init__.py`: Version 1.7.6

## 📦 Bağımlılıklar

### Yeni Bağımlılıklar
- `scipy>=1.9.0` - Convex Hull hesaplamaları için

## 🚀 Kullanım

Bu release'i kullanmak için:

```bash
# Belirli bir versiyona geç
git checkout v1.7.6

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.6
```

### Formasyon Sistemi

Formasyon pozisyonları artık liderin yaw açısına göre dinamik olarak döndürülüyor:

```python
from FiratROVNet.gnc import Filo

filo = Filo()

# Formasyon oluştur (liderin yaw açısına göre otomatik döndürülür)
filo.formasyon("V_SHAPE", aralik=20)

# Otomatik formasyon seçimi (hiyerarşik arama)
formasyon_id = filo.formasyon_sec(margin=30)
```

### Yeni Kullanım Kılavuzu

Detaylı kullanım bilgileri için:
- `KILAVUZ/KULLANIM_KILAVUZU.md`: Ana kullanım kılavuzu

## 🔄 Geriye Uyumluluk

Tüm değişiklikler geriye uyumludur. Mevcut kodlar çalışmaya devam eder, ancak formasyon sistemi artık daha akıllı ve dinamik çalışıyor.

## 📚 İlgili Dokümantasyon

- Ana kullanım kılavuzu: `KILAVUZ/KULLANIM_KILAVUZU.md`
- Senaryo modülü: `KILAVUZ/SENARYO_KULLANIM.md`
- Konsol erişimi: `KILAVUZ/KONSOL_ERISIM.md`

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team

