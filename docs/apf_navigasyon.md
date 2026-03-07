# 🌊 APF Navigasyon

**Yapay Potansiyel Alan (APF)** tabanlı navigasyon, Fırat-GNC’de hedef çekimi ve engel itmesi ile ROV’lar için yerel hareket vektörü üretir.

---

## 1. Kısa Özet

- **Hedef**: Çekim potansiyeli; hedefe doğru birim vektör katkısı.
- **Engeller**: İtme potansiyeli; engellere yaklaştıkça ters yönde kuvvet.
- **Çıktı**: Dünya koordinatında bir **hedef yön vektörü**; bu vektör [Motor ve İtki Sistemi](./motor_tasarimi.md) ile motor güçlerine dağıtılır.

Detaylı formüller, parametreler ve diyagramlar bu sayfaya ileride eklenecektir (potansiyel fonksiyonları, mesafe eşikleri, normalize edilmiş toplam vektör).

---

## 2. GNC ile Bağlantı

APF çıktısı → `tum_motorlarin_guclerini_hesapla(hedef_vektor_dunya=...)` ve `tork_gucleri_hesapla(...)` ile motor komutlarına dönüştürülür. Akış için [GNC Mimarisi](./gnc_mimari.md) ve [Motor ve İtki Sistemi](./motor_tasarimi.md) dokümanlarına bakınız.

---

*Fırat Üniversitesi – Otonom Sistemler & Yapay Zeka Laboratuvarı*
