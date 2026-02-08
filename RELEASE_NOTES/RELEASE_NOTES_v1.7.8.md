# Release Notes v1.7.8

## 📅 Tarih
Şubat 2026

## 🎯 Özet
Bu sürüm, Minimap üzerinden dinamik hedef belirleme, ID tabanlı hedef kuyruğu yönetimi, gelişmiş 3D görselleştirme ve Git deposu boyut optimizasyonlarını içermektedir.

## ✨ Yeni Özellikler / İyileştirmeler

### Minimap ve Etkileşim
- **Tıkla-Git Özelliği**: Minimap üzerine sol tıklanarak İHA/ROV için anlık hedefler belirlenebilir hale getirildi.
- **Dinamik 2D İkonlar**: Belirlenen her hedef, Minimap üzerinde kendi ID numarası ve özel rengiyle (Cyan) anlık olarak işaretlenir.
- **Koordinat Dönüşümü**: Minimap üzerindeki yerel tıklama koordinatları, 400m'lik havuz ölçeğine otomatik olarak senkronize edildi.

### Hedef Yönetimi ve Kuyruk Sistemi (Navigation Queue)
- **ID Tabanlı Takip**: Her hedefe benzersiz bir ID atanarak görev takibi kolaylaştırıldı.
- **Sıralı Hedef Takibi**: Lider ROV için bir hedef kuyruğu (`nav_queue`) oluşturuldu. ROV, bir hedefi tamamladığında otomatik olarak sıradaki hedefe yönlenir.
- **Otomatik Temizlik**: ROV hedefe vardığında, hem 3D dünyadaki görsel işaretçi hem de Minimap üzerindeki ikon otomatik olarak imha edilir.

### 3D Görselleştirme (Waypoint Visualizer)
- **Gelişmiş İşaretçiler**: Hedef noktalarında yer seviyesinde çemberler ve üzerinde Billboard (kameraya sürekli dönen) ID metinleri eklendi.
- **Debug Modu**: Geçici hedefler için kırmızı X işareti ve yeşil halka içeren "Debug" modu korundu.

### Depo (Repository) ve Performans
- **Git History Temizliği**: Depo boyutunu aşırı şişiren ve push hatalarına neden olan büyük 3D varlık geçmişi temizlendi.
- **Models-3D Optimizasyonu**: Büyük modeller parçalı commit yapısıyla sisteme dahil edildi; 100MB sınırını aşan `Bluerov2.glb` gibi dosyalar `.gitignore` kapsamına alınarak depo sağlığı korundu.

## 🐛 Hata Düzeltmeleri
- **Push Reddedilme Sorunu**: Geçmişteki büyük dosya kalıntıları "Orphan Branch" yöntemiyle temizlenerek push hataları giderildi.
- **Merge Conflicts**: Farklı geçmişlere sahip dalların birleşimi sırasında oluşan çakışmalar (`allow-unrelated-histories`) giderildi.
- **Update Döngüsü Senkronizasyonu**: `set_update_function` isimlendirme hataları giderilerek sistemin stabil çalışması sağlandı.

## 📝 Değişiklikler

### Dosya Değişiklikleri
- `FiratROVNet/kutuphane/helper/gnc_helper.py`: `hedef_gorsel_olustur` ve `hedef_sil` fonksiyonları Minimap tetikleyicileriyle güncellendi.
- `FiratROVNet/simulasyon.py`: `Minimap` sınıfına `hedef_isaretle` ve `hedef_sil` metodları eklendi.
- `main.py`: Navigasyon kuyruğu, mouse input yönetimi ve otomatik hedef silme mantığı eklendi.
- `.gitignore`: Büyük 3D binary dosyaları engelleme listesine alındı.

## 🚀 Kullanım

```bash
# v1.7.8 sürümüne geç
git checkout v1.7.8
