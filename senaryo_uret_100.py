from FiratROVNet import senaryo

# İlk kurulum (yavaş - ortam oluşturulur)
senaryo.uret(n_rovs=4, n_engels=10)

# Hızlı pozisyon güncelleme (çok hızlı - sadece pozisyonlar değişir)
for episode in range(100):
    senaryo.uret()  # Aynı sayılar, farklı koordinatlar
    # Eğitim adımları...