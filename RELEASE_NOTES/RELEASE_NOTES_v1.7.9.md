# Release Notes v1.7.9 — Latest

## 📅 Tarih
1 Mart 2026

## 🎯 Özet
Bu sürüm, **develop** dalındaki tüm güncel değişikliklerin **main** ile birleştirilmesiyle oluşturulmuş **güncel kararlı (latest)** sürümdür. Tip güvenliği, kamera refaktörü, yüzey bilgisi/motor güvenliği, şema dışa aktarımı ve yapılandırma iyileştirmeleri içerir.

## ✨ Öne Çıkan Özellikler

- **Tip güvenliği ve None kontrolleri**: `control`, `damage_system`, `gnc_helper` (core, data, geometry, visualization) ve `motor` modüllerinde `filo_ref` ve `None` kontrolleri.
- **Kamera yöneticisi**: Panda3D tabanlı tek DisplayRegion kullanımı; Ursina bağımlılığı kaldırıldı.
- **Yüzey bilgisi (yuzey_bilgileri)** ve **motorlari_calistir** güvenlik kontrolleri.
- **Şema dışa aktarımı**: `gnc/schema_export.py` ve `SCHEMA/` (ROV0 bilgi + motor şema PDF) eklendi.
- **Config / main / hull**: Yapılandırma ve ana giriş noktası güncellemeleri; `hull_information.json` güncellendi.
- **Pyright**: `pyrightconfig.json` ile statik tip kontrolü yapılandırması.

## 📝 Teknik Değişiklikler

### GNC & Helper
- `FiratROVNet/gnc/damage_system.py`: None ve referans kontrolleri.
- `FiratROVNet/gnc/motor.py`: Yeni modül; motor güvenlik ve sürüş mantığı.
- `FiratROVNet/kutuphane/helper/gnc_helper/control.py`: Tip güvenliği, `filo_ref` kontrolleri.
- `FiratROVNet/kutuphane/helper/gnc_helper/core.py`, `mixins/data.py`, `mixins/geometry.py`, `mixins/visualization.py`: None/guard ve `apf_guncelle_tum` ile uyum.

### Simülasyon & Kamera
- `FiratROVNet/camera_manager.py`: Panda3D DisplayRegion odaklı refaktör.
- `FiratROVNet/kutuphane/helper/EntityLoader.py`: Ursina import ve ROV yükleme uyumu.
- `FiratROVNet/simulasyon.py`: ROV() ve lint düzeltmeleri.

### Şema & Config
- `FiratROVNet/gnc/__init__.py`: yuzey_bilgileri, motorlari_calistir, kamera_ayarla entegrasyonu.
- `FiratROVNet/gnc/schema_export.py`: Yeni şema dışa aktarım modülü.
- `SCHEMA/ROV0/bilgi.json`, `SCHEMA/ROV0/rov_motor_sema.pdf`: ROV0 şema varlıkları.
- `FiratROVNet/config.py`, `main.py`, `hull_information.json`: Güncel ayarlar ve veri.
- `pyrightconfig.json`: Proje kökünde Pyright yapılandırması.

## 🐛 Hata Düzeltmeleri

- GNC ve kontrol kodunda `None` ve eksik `filo_ref` kullanımına bağlı hataların önlenmesi.
- Kamera yönetiminde Ursina/Panda3D uyumsuzluklarının giderilmesi.

## 🚀 İndirme ve Kurulum

### Kaynak kodu (Zip)
- **FiratRovNet-v1.7.9.zip**: Bu sürümün kaynak kodu arşivi (GitHub Release ekinde).

### Git ile
```bash
git clone --depth 1 --branch v1.7.9 https://github.com/FiratROVNet/FiratRovNet.git
cd FiratRovNet
```

### Pip ile (GitHub)
```bash
pip install git+https://github.com/FiratROVNet/FiratRovNet.git@v1.7.9
```

## 🔄 Geriye Uyumluluk

- Kamera API’si Panda3D odaklı kullanıma geçti; doğrudan Ursina kamera kullanımı kaldırıldı.
- GNC ve helper çağrıları mevcut kullanım ile uyumludur; ek `filo_ref`/None kontrolleri davranışı değiştirmez.

## 📚 İlgili Dokümantasyon

- `README.md`: Genel proje ve kurulum.
- `SCHEMA/README.md`: Şema yapısı ve kullanımı.
- `RELEASE_NOTES/`: Diğer sürüm notları.

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team
