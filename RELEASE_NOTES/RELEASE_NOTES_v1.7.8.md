# Release Notes v1.7.8

## 📅 Tarih: 09 February 2026

## 🎯 Özet
Bu sürüm, Minimap üzerinden dinamik hedef yönetimi, ID tabanlı kuyruk sistemi ve depo optimizasyonu içeren kapsamlı bir güncellemedir.

## ✨ Öne Çıkan Özellikler
- **Minimap İnteraktivitesi**: Haritaya tıklayarak anlık rota belirleme (Tıkla-Git).
- **Hedef Kuyruğu (Queue)**: Lider İHA/ROV için sıralı hedef takip mekanizması.
- **ID Tabanlı Takip**: Hem 3D dünyada hem Minimap'te ID'li görsel işaretçiler.
- **Otomatik Temizlik**: Varılan hedeflerin görsellerinin otomatik imha edilmesi.
- **Depo Sağlığı**: Büyük 3D varlıkların temizlenmesi ve .gitignore optimizasyonu.

## 📝 Teknik Değişiklikler
- gnc_helper.py ve simulasyon.py modülleri Minimap tetikleyicileriyle senkronize edildi.
- Navigasyon kuyruğu yönetimi ana döngüye entegre edildi.
