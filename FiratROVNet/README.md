# FiratROVNet — Çekirdek Kütüphane

Fırat-GNC simülasyonunun çekirdek Python kütüphanesi. Güdüm, Navigasyon ve Kontrol (GNC), 3D ortam, kamera yönetimi, GAT analizi ve akustik iletişim simülasyonunu içerir.

## Ana Modüller

| Modül | Açıklama |
|-------|----------|
| `gnc` | Filo, TemelGNC, motor konfigürasyonu, formasyon, navigasyon kuyruğu |
| `simulasyon` | Ortam, `sim_olustur`, minimap, fizik, 3D render |
| `config` | Canlı ayarlar (`cfg`), GAT/sensör/havuz/motor sabitleri |
| `camera_manager` | ROV FPV kamera ve ekran bölgeleri (Panda3D) |
| `senaryo` | Headless senaryo ve GAT veri üretimi |
| `iletisim` | Akustik modem simülatörü |
| `hull` | Convex hull ve güvenlik çevresi |
| `a_star` | A* yol planlama |

## Kullanım

Ana uygulama ve konsol komutları için proje kökündeki [README.md](../README.md) ve [KILAVUZ/KULLANIM_KILAVUZU.md](../KILAVUZ/KULLANIM_KILAVUZU.md) dosyalarına bakın.

## Geliştirici

Ömer Faruk Çelik — Fırat Üniversitesi, Otonom Sistemler & Yapay Zeka Laboratuvarı
