"""
Simülasyon Yardımcı Modülü
===========================
Bu modül simulasyon.py içindeki yardımcı fonksiyonları içerir.
Kodun okunaklılığını ve bakımını kolaylaştırmak için oluşturulmuştur.
"""

import random
from ursina import Entity, Vec3, color


# ============================================================
# KOORDİNAT DÖNÜŞÜM FONKSİYONLARI
# ============================================================

def sim_to_ursina(x_2d, y_2d, z_depth):
    """
    Simülasyon koordinat sisteminden Ursina koordinat sistemine dönüşüm.
    
    Bu simülasyonda kullanılan koordinat sistemi:
    - X ekseni: 2D düzlemde yatay (horizontal - birinci boyut)
    - Y ekseni: 2D düzlemde dikey (horizontal - ikinci boyut)
    - Z ekseni: Derinlik (depth) - pozitif değerler yüzeye yakın, negatif değerler derinlik
    
    Ursina engine'in kendi koordinat sistemi:
    - Ursina X: horizontal (sağ-sol)
    - Ursina Y: vertical (yukarı-aşağı)
    - Ursina Z: depth (ileri-geri)
    
    Args:
        x_2d: 2D düzlemde yatay (horizontal - birinci boyut)
        y_2d: 2D düzlemde dikey (horizontal - ikinci boyut)
        z_depth: Derinlik (depth)
    
    Returns:
        tuple: (ursina_x, ursina_y, ursina_z) Ursina koordinatları
    """
    return (x_2d, z_depth, y_2d)


def ursina_to_sim(ursina_x, ursina_y, ursina_z):
    """
    Ursina koordinat sisteminden simülasyon koordinat sistemine dönüşüm.
    
    Args:
        ursina_x: Ursina X (horizontal)
        ursina_y: Ursina Y (vertical)
        ursina_z: Ursina Z (depth)
    
    Returns:
        tuple: (x_2d, y_2d, z_depth) Simülasyon koordinatları
    """
    return (ursina_x, ursina_z, ursina_y)


# ============================================================
# KAYA OLUŞTURMA FONKSİYONLARI
# ============================================================

def kaya_boyutlari_uret(min_boyut=15, max_boyut=40, max_z_boyut=60):
    """
    Rastgele kaya boyutları üretir.
    
    Args:
        min_boyut: Minimum boyut (varsayılan: 15)
        max_boyut: Maksimum boyut (varsayılan: 40)
        max_z_boyut: Maksimum Z boyutu (varsayılan: 60)
    
    Returns:
        tuple: (s_x, s_y, s_z) kaya boyutları
    """
    s_x = random.uniform(min_boyut, max_boyut)
    s_y = random.uniform(min_boyut, max_boyut)
    s_z = random.uniform(min_boyut, max_z_boyut)
    return (s_x, s_y, s_z)


def kaya_yari_cap_hesapla(s_x, s_y, s_z):
    """
    Kaya yarıçapını hesaplar (en büyük boyutun yarısı).
    
    Args:
        s_x: X boyutu
        s_y: Y boyutu
        s_z: Z boyutu
    
    Returns:
        float: Kaya yarıçapı
    """
    return max(s_x, s_y, s_z) / 2.0


def kaya_guvenli_pozisyon_bul(
    havuz_genisligi,
    kaya_yari_cap,
    guvenlik_mesafesi=8.0,
    mevcut_kayalar=None,
    max_deneme=100
):
    """
    Kaya için güvenli pozisyon bulur.
    Havuz sınırlarına değmeyecek ve çaplarıyla orantılı güvenlik mesafesiyle içerde olacak şekilde.
    
    Args:
        havuz_genisligi: Havuz genişliği (+-havuz_genisligi)
        kaya_yari_cap: Kaya yarıçapı
        guvenlik_mesafesi: Güvenlik mesafesi (metre, varsayılan: 8.0)
        mevcut_kayalar: Mevcut kayaların pozisyonları [(x, z), ...] (opsiyonel)
        max_deneme: Maksimum deneme sayısı (varsayılan: 100)
    
    Returns:
        tuple: (x, z) güvenli pozisyon veya None
    """
    # Havuz sınırları
    havuz_sinir = havuz_genisligi
    
    # Toplam güvenlik mesafesi = kaya yarıçapı + güvenlik mesafesi
    toplam_guvenlik = kaya_yari_cap + guvenlik_mesafesi
    
    # Güvenli alan sınırları (havuz sınırlarından içerde)
    min_x = -havuz_sinir + toplam_guvenlik
    max_x = havuz_sinir - toplam_guvenlik
    min_z = -havuz_sinir + toplam_guvenlik
    max_z = havuz_sinir - toplam_guvenlik
    
    # Eğer güvenli alan çok küçükse, uyarı ver ve minimum değerleri kullan
    if min_x >= max_x or min_z >= max_z:
        # Çok küçük alan, minimum güvenlik mesafesiyle dene
        toplam_guvenlik = kaya_yari_cap + 1.0  # Minimum 1 metre güvenlik
        min_x = -havuz_sinir + toplam_guvenlik
        max_x = havuz_sinir - toplam_guvenlik
        min_z = -havuz_sinir + toplam_guvenlik
        max_z = havuz_sinir - toplam_guvenlik
    
    # Mevcut kayalar varsa, onlardan da uzak olmalı
    if mevcut_kayalar:
        for deneme in range(max_deneme):
            # Rastgele pozisyon dene
            x = random.uniform(min_x, max_x)
            z = random.uniform(min_z, max_z)
            
            # Mevcut kayalardan yeterince uzak mı kontrol et
            yeterince_uzak = True
            for mevcut_x, mevcut_z, mevcut_yari_cap in mevcut_kayalar:
                # İki kaya arasındaki mesafe
                dx = x - mevcut_x
                dz = z - mevcut_z
                mesafe = (dx**2 + dz**2)**0.5
                
                # Minimum mesafe = iki kaya yarıçapı toplamı + güvenlik mesafesi
                min_mesafe = kaya_yari_cap + mevcut_yari_cap + guvenlik_mesafesi
                
                if mesafe < min_mesafe:
                    yeterince_uzak = False
                    break
            
            if yeterince_uzak:
                return (x, z)
    else:
        # Mevcut kaya yoksa, direkt rastgele pozisyon döndür
        x = random.uniform(min_x, max_x)
        z = random.uniform(min_z, max_z)
        return (x, z)
    
    # Güvenli pozisyon bulunamadıysa, en uzak noktayı seç
    if mevcut_kayalar:
        # Mevcut kayaların ortalamasından uzak bir nokta bul
        ortalama_x = sum(k[0] for k in mevcut_kayalar) / len(mevcut_kayalar)
        ortalama_z = sum(k[1] for k in mevcut_kayalar) / len(mevcut_kayalar)
        
        # Ortalamadan uzak bir pozisyon seç
        if abs(ortalama_x) < havuz_sinir / 2:
            x = havuz_sinir * 0.8 if ortalama_x < 0 else -havuz_sinir * 0.8
        else:
            x = -ortalama_x * 0.5
        
        if abs(ortalama_z) < havuz_sinir / 2:
            z = havuz_sinir * 0.8 if ortalama_z < 0 else -havuz_sinir * 0.8
        else:
            z = -ortalama_z * 0.5
        
        # Sınırları kontrol et
        x = max(min_x, min(max_x, x))
        z = max(min_z, min(max_z, z))
        
        return (x, z)
    
    # Son çare: Merkezden uzak bir nokta
    return (random.choice([min_x, max_x]), random.choice([min_z, max_z]))


def kaya_y_pozisyon_hesapla(s_y, sea_floor_y, water_surface_y_base):
    """
    Kaya için Y (dikey) pozisyonunu hesaplar.
    Tabanı deniz tabanına değmeli, üstü su yüzeyinin altında olmalı.
    
    Args:
        s_y: Kaya Y boyutu (yükseklik)
        sea_floor_y: Deniz tabanı Y koordinatı
        water_surface_y_base: Su yüzeyi Y koordinatı
    
    Returns:
        float: Kaya Y pozisyonu
    """
    # Kaya pozisyonu: Tabanı deniz tabanına değmeli, üstü su yüzeyinin altında olmalı
    kaya_alt_sinir = sea_floor_y  # Kayanın alt kısmı deniz tabanında
    kaya_ust_sinir = water_surface_y_base - (s_y / 2) - 2  # Kayanın üst kısmı su yüzeyinin 2 birim altında
    
    # Eğer kaya çok büyükse ve su yüzeyine sığmıyorsa, deniz tabanına yerleştir
    if kaya_ust_sinir < kaya_alt_sinir:
        y = sea_floor_y + (s_y / 2)  # Tabanı deniz tabanında
    else:
        y = random.uniform(kaya_alt_sinir, kaya_ust_sinir)
    
    return y


def kaya_olustur(
    x, y, z,
    s_x, s_y, s_z,
    sea_floor_y
):
    """
    Kaya Entity'si oluşturur.
    
    Args:
        x: X pozisyonu
        y: Y pozisyonu
        z: Z pozisyonu
        s_x: X boyutu
        s_y: Y boyutu
        s_z: Z boyutu
        sea_floor_y: Deniz tabanı Y koordinatı (kaya tabanı için)
    
    Returns:
        Entity: Oluşturulan kaya Entity'si
    """
    # Kaya rengi (gri tonları)
    gri = random.randint(80, 100)
    kaya_rengi = color.rgb(gri, gri, gri)
    
    # Kaya Entity'si oluştur
    engel = Entity(
        model='icosphere',
        color=kaya_rengi,
        texture='noise',
        scale=(s_x, s_y, s_z),
        position=(x, sea_floor_y, z),  # Tabanı deniz tabanında
        rotation=(random.randint(0, 360), random.randint(0, 360), random.randint(0, 360)),
        collider='sphere',  # Performans için küre collider yeterli
        unlit=True
    )
    
    return engel


def kayalari_olustur(
    n_engels,
    havuz_genisligi,
    sea_floor_y,
    water_surface_y_base,
    guvenlik_mesafesi=8.0,
    min_boyut=15,
    max_boyut=40,
    max_z_boyut=60
):
    """
    Tüm kayaları oluşturur ve güvenli pozisyonlara yerleştirir.
    
    Args:
        n_engels: Oluşturulacak kaya sayısı
        havuz_genisligi: Havuz genişliği
        sea_floor_y: Deniz tabanı Y koordinatı
        water_surface_y_base: Su yüzeyi Y koordinatı
        guvenlik_mesafesi: Güvenlik mesafesi (metre, varsayılan: 8.0)
        min_boyut: Minimum kaya boyutu (varsayılan: 15)
        max_boyut: Maksimum kaya boyutu (varsayılan: 40)
        max_z_boyut: Maksimum Z boyutu (varsayılan: 60)
    
    Returns:
        list: Oluşturulan kaya Entity'leri listesi
    """
    engeller = []
    mevcut_kayalar = []  # [(x, z, yari_cap), ...]
    
    for _ in range(n_engels):
        # Kaya boyutları
        s_x, s_y, s_z = kaya_boyutlari_uret(min_boyut, max_boyut, max_z_boyut)
        
        # Kaya yarıçapı
        kaya_yari_cap = kaya_yari_cap_hesapla(s_x, s_y, s_z)
        
        # Güvenli pozisyon bul
        pozisyon = kaya_guvenli_pozisyon_bul(
            havuz_genisligi=havuz_genisligi,
            kaya_yari_cap=kaya_yari_cap,
            guvenlik_mesafesi=guvenlik_mesafesi,
            mevcut_kayalar=mevcut_kayalar,
            max_deneme=100
        )
        
        if pozisyon is None:
            # Pozisyon bulunamadıysa, uyarı ver ve devam et
            print(f"⚠️ Kaya için güvenli pozisyon bulunamadı, atlanıyor...")
            continue
        
        x, z = pozisyon
        
        # Y pozisyonu hesapla
        y = kaya_y_pozisyon_hesapla(s_y, sea_floor_y, water_surface_y_base)
        
        # Kaya oluştur
        engel = kaya_olustur(x, y, z, s_x, s_y, s_z, sea_floor_y)
        
        # Listeye ekle
        engeller.append(engel)
        mevcut_kayalar.append((x, z, kaya_yari_cap))
    
    return engeller

