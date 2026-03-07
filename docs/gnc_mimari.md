# 🧠 GNC Mimarisi

Fırat-GNC'de **Güdüm, Navigasyon ve Kontrol (GNC)** yapısı: Filo, ROV entity'leri, motorlar ve APF/hedef birimi tek bir katmanda koordine edilir.

---

## 1. Katman Özeti

| Katman | Bileşen | Açıklama |
|--------|---------|----------|
| Filo | Filo, FiloHelper | Motor kütüphaneleri (birim vektör, tork_bv), yerel/dünya dönüşümleri, hedef ve kuyruk |
| ROV | Entity + gnc, motorlar | Pozisyon, rotasyon, fizik düğümü, tek ROV GNC state |
| Motor | Motor, calistir | Yerel yön to dünya kuvvet/tork, applyCentralForce / applyTorque |
| APF / Hedef | tum_motorlarin_guclerini_hesapla, tork_gucleri_hesapla | Hedef vektör to yerel, skaler çarpım (itki) ve tork dağılımı |

---

## 2. Veri Akışı (Özet)

1. **Hedef** (dünya): APF veya kullanıcı komutu to dünya yön vektörü.
2. **Dünya to Yerel**: dunya_to_yerel_vektor ile ROV eksenlerinde izdüşüm.
3. **İtki güçleri**: Yerel hedef ile motor birim vektörlerinin skaler çarpımı.
4. **Tork güçleri**: Dünya tork (yatay cross) to yerel tork, motor tork vektörleri ile skaler çarpım.
5. **Fizik**: Her motor calistir(guc) ile kuvvet ve torku dünya ekseninde uygular.

Detaylı formüller ve motor konfigürasyonu için: [Motor ve İtki Sistemi](./motor_tasarimi.md).

---

## 3. Genişletilebilir Yapı

- Yeni **motor konfigürasyonları**: BlueROV2 benzeri yeni şemalar eklenebilir; motorlar_bv ve tork_bv güncellenir.
- Yeni **GNC modülleri**: Aynı hedef vektörü arayüzü ile farklı kontrol yasaları (APF, A*, formasyon) beslenebilir.
- **Şema/JSON**: SCHEMA/ ve schema_export.py ile yeni ROV tipleri dokümante edilebilir.

---

*Ömer Faruk Çelik — Fırat Üniversitesi, Otonom Sistemler ve Yapay Zeka Laboratuvarı*
