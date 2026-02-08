# Release Notes v1.7.7

## 📅 Tarih
Ocak 2025

## 🎯 Özet
Bu sürüm, hedefe giderken ROV çember çizmesini önleyen GNC iyileştirmeleri, filo.move hız düzeltmesi, hull formasyon koordinat düzeltmesi ve ortam modülünün kaldırılması ile GAT eğitim akışının sadeleştirilmesini içermektedir.

## ✨ Yeni Özellikler / İyileştirmeler

### GNC – Hedefe Gitme (filo.git / filo.git_path)
- **Çember çizmesini önleme**: Hedefe yaklaşırken teğetsel hız sönümlemesi eklendi
  - Yakın mesafede (mesafe < 2 m) hız vektörü radyal ve teğetsel bileşenlere ayrılıyor; teğetsel bileşen kademeli sönümleniyor
  - mesafe < 0.5 m: teget_carpani 0.04 (neredeyse tam radyal, doğrudan hedefe)
  - mesafe < 1.0 m: teget_carpani 0.08
  - 1–2 m: teget_carpani 0.16
  - ROV artık hedef etrafında dönmek yerine doğrudan hedefe giriyor
- **filo.git_path hızı**: Hedefe giderken güç çarpanı 0.4 → 0.28 (daha kontrollü yaklaşma)
- **Hedef yokken filo.move hızı**: Hedef yokken GNC guncelle() her frame velocity *= 0.4 uyguluyordu; filo.move() ile manuel sürüş varken bu sönümleme yapılmıyor, böylece filo.move ile normal hızda hareket ediliyor

### Hull – Formasyon Geçerliliği
- **formasyon_gecerli_mi** 2D koordinat düzeltmesi: test_points Ursina formatında (x, z, y); hull 2D yatay düzlemde (sim_x, sim_y) ile tanımlı. Nokta kontrolü (tp[0], tp[2]) ve mesafe hesabı yatay düzlemde yapılıyor.

### Senaryo
- ROV ekleme/çıkarma sırasında **sessiz mod** (_sessiz_mod) kullanımı
- **Su yüzeyi animasyonu**: senaryo.guncelle() içinde ocean_surface.update() çağrısı
- Ursina time.dt ayarı: `from ursina import time` ile delta_time senkronizasyonu

### GAT ve Bağımlılıklar
- **ortam.py kaldırıldı**: Veri üretimi artık senaryo tabanlı filo.gat_veri_uret() ile yapılıyor
- GAT eğitimi: `python -m GAT.gat_train [--epochs 200] [--no-resume]`
- **requirements.txt**: Ursina 7.0.0 sabit (ursina==7.0.0); su yüzeyi/texture uyumluluğu

## 🐛 Hata Düzeltmeleri

- **filo.move yavaşlığı**: Hedef yokken velocity *= 0.4 uygulandığı için filo.move ile hareket iki kat yavaş hissediliyordu; manuel kuvvet (active_forces) varken sönümleme atlanıyor
- **filo.git çember**: Hedefe yakınken hız sadece büyüklük olarak sınırlanıyordu, yön hedefe zorlanmıyordu; teğetsel momentum orbit oluşturuyordu; yakın mesafede teğetsel sönümleme ile düzeltildi
- **formasyon_gecerli_mi**: 3D nokta ile 2D hull kontrolü hatası; 2D nokta (sim_x, sim_y) ve yatay mesafe kullanımına geçildi

## 📝 Değişiklikler

### Dosya Değişiklikleri
- `FiratROVNet/kutuphane/helper/gnc_helper.py`:
  - guncelle(): hedef yokken manuel_kuvvet_var ise velocity *= 0.4 uygulanmıyor
  - guncelle(): HEDEF_GUC_CARPANI = 0.28, vektor_to_motor_sim sonrası yakın mesafede teğetsel sönümleme (radyal/teğetsel ayrıştırma, teget_carpani ile sönüm)
- `FiratROVNet/hull.py`: formasyon_gecerli_mi 2D nokta ve mesafe hesabı
- `FiratROVNet/__init__.py`: __version__ = "1.7.7", ortam/veri_uret kaldırıldı
- `FiratROVNet/senaryo.py`: _sessiz_mod, ocean_surface.update(), ursina time.dt, ROV sayısı değişmediğinde sadece pozisyon güncelleme
- `requirements.txt`: ursina==7.0.0
- `README.md`, `run_tests.py`: ortam/veri_uret referansları kaldırıldı veya _test_veri_uret ile değiştirildi

## 🚀 Kullanım

```bash
# Belirli bir versiyona geç
git checkout v1.7.7

# veya pip ile yükle
pip install git+https://github.com/FiratROVNet/FiratROVNet.git@v1.7.7
```

### Hedefe Gitme
- `filo.git(rov_id, x, y, z)` ve `filo.git_path(rov_id, hedef)` artık hedefe yaklaşırken çember çizmeden doğrudan hedefe giriyor
- `filo.move(rov_id, yon, guc)` hedef yokken normal hızda çalışıyor

## 🔄 Geriye Uyumluluk

- `from FiratROVNet import ortam`, `veri_uret` kullanımı kaldırıldı; GAT eğitimi için `filo.gat_veri_uret()` ve `python -m GAT.gat_train` kullanın
- Diğer değişiklikler geriye uyumludur

## 📚 İlgili Dokümantasyon

- KILAVUZ/KULLANIM_KILAVUZU.md
- KILAVUZ/SENARYO_KULLANIM.md
- GAT/README.md

## 🙏 Katkıda Bulunanlar
FiratROVNet Development Team
