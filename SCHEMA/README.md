# SCHEMA — ROV Motor Şemaları

Bu klasör, her ROV tipi için **motor konfigürasyon şemaları** ve **veri dosyalarını** tutar. Klasör isimleri ROV tanımlayıcısıdır (örn. ROV0, ROV1).

## Klasör yapısı

Her ROV için bir alt klasör açın; klasör adı simülasyondaki ROV tipini/kinematiğini temsil eder. Her klasörde şu iki dosya **zorunludur**:

| Dosya | Açıklama |
|-------|----------|
| rov_motor_sema.pdf | Motor yerleşimi ve itki yönlerini gösteren şema (yayında/makalede kullanılabilir). |
| bilgi.json | Motor konumları ve birim yön vektörleri (kod ve dokümantasyonla uyumlu). |

Yeni bir ROV eklediğinizde:

1. `SCHEMA/ROV<id>/` klasörünü oluşturun.
2. İçine `rov_motor_sema.pdf` ve `bilgi.json` ekleyin.
3. Listeyi güncellemek için: `python SCHEMA/update_readme.py` çalıştırın (aşağıdaki tablo otomatik doldurulur).

## Mevcut ROV şemaları

*Aşağıdaki tablo `update_readme.py` ile otomatik üretilir.*

| ROV | Motor şeması (PDF) | Veri (JSON) |
|-----|-------------------|--------------|
| ROV0 | [rov_motor_sema.pdf](ROV0/rov_motor_sema.pdf) | [bilgi.json](ROV0/bilgi.json) |

---
## bilgi.json formatı

Her ROV klasöründeki `bilgi.json`, aşağıdaki yapıda motor konum ve yön vektörlerini içerir (Fırat-GNC `schema_export` ile uyumludur):

```json
{
  "rov_id": 0,
  "motorlar": [
    { "name": "m0", "position": [-200, 0, 200], "direction": [0.707, 0, 0.707] },
    ...
  ]
}
```

- **position**: Yerel koordinatta motor konumu (model birimleri).
- **direction**: Birim itki yön vektörü (yerel, normalize).

Detaylı motor kinematiği ve formüller için: [Motor ve İtki Sistemi](../docs/motor_tasarimi.md).
