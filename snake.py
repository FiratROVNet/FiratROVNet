import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString, Polygon, MultiPolygon
from shapely.ops import unary_union, nearest_points
import alphashape

def generate_smart_contour(valid_points, forbidden_points, alpha=1.0, buffer_radius=0.05, channel_width=0.02):
    """
    Yasaklı noktaları dışarıda bırakacak şekilde dış konturu içeri büker (indentation).
    """
    # 1. Mevcut noktalarla en iyi dış kabuğu (Alpha Shape) oluştur
    points_cloud = [(p[0], p[1]) for p in valid_points]
    base_shape = alphashape.alphashape(points_cloud, alpha)
    
    # Alpha shape bazen MultiPolygon döner, en büyük parçayı alalım
    if isinstance(base_shape, MultiPolygon):
        base_shape = max(base_shape.geoms, key=lambda a: a.area)
    
    # Sadece dış kabuğu al (iç delikleri varsa temizle)
    if isinstance(base_shape, Polygon):
        base_shape = Polygon(base_shape.exterior)
    
    final_shape = base_shape
    
    # 2. Her yasaklı nokta için işlem yap
    for fp in forbidden_points:
        p_obj = Point(fp)
        
        # Yasaklı nokta zaten dışarıdaysa dokunma
        if not final_shape.contains(p_obj):
            continue
            
        # A. Yasaklı bölge (Güvenlik çemberi)
        forbidden_zone = p_obj.buffer(buffer_radius)
        
        # B. KANAL AÇMA: Noktadan dış sınıra en kısa yolu bul
        # Dış sınır çizgisi
        exterior_line = final_shape.exterior
        
        # En yakın noktaları bul (biri bizim noktamız, diğeri sınırda)
        p1, p2 = nearest_points(forbidden_zone, exterior_line)
        
        # Bu iki nokta arasına bir çizgi (kanal) çek ve kalınlaştır
        channel_line = LineString([p_obj, p2])
        channel_poly = channel_line.buffer(channel_width) # Kanal genişliği
        
        # C. Hem çemberi hem kanalı ana şekilden çıkar
        cut_area = unary_union([forbidden_zone, channel_poly])
        final_shape = final_shape.difference(cut_area)
        
        # İşlem sonrası şekil parçalanmış olabilir (MultiPolygon), 
        # en büyük parçayı (ana kıta) koru, kopan adaları at.
        if isinstance(final_shape, MultiPolygon):
            final_shape = max(final_shape.geoms, key=lambda a: a.area)

    return final_shape

# --- TEST VERİSİ ---
np.random.seed(42) # Her seferinde aynı sonuç için

# Mavi Noktalar (Alan)
points = np.random.rand(80, 2)

# Siyah Noktalar (Yasaklılar - Rastgele içerilere serpiştirilmiş)
forbidden_points = np.array([
    [0.5, 0.5],   # Tam ortada
    [0.2, 0.8],   # Sol üstte
    [0.8, 0.3],    # Sağ altta
    [0.3, 0.5],   # Tam ortada
    [0.8, 0.8],   # Sol üstte
    [0.1, 0.3]    # Sağ altta
    
])

# --- HESAPLAMA ---
# alpha: Şeklin ne kadar sıkı sarılacağı (düşük = gevşek/konveks, yüksek = girintili)
# buffer_radius: Yasaklı noktanın etrafındaki boşluk
# channel_width: Dışarı açılan yolun genişliği
result_shape = generate_smart_contour(
    points, 
    forbidden_points, 
    alpha=2.0,           
    buffer_radius=0.06,  
    channel_width=0.03
)

# --- ÇİZİM ---
plt.figure(figsize=(8, 8))

# 1. Alanı temsil eden mavi noktalar
plt.scatter(points[:,0], points[:,1], c='tab:blue', label='Alan Noktaları')

# 2. Yasaklı siyah noktalar
plt.scatter(forbidden_points[:,0], forbidden_points[:,1], c='black', s=100, zorder=5, label='Yasaklı Noktalar')

# 3. Hesaplanan Kırmızı Sınır (Counter)
if isinstance(result_shape, Polygon):
    x, y = result_shape.exterior.xy
    plt.plot(x, y, color='red', linewidth=2, label='Yeni Kontur')
    plt.fill(x, y, color='red', alpha=0.1) # İçini hafif boya

plt.legend()
plt.title("Yasaklı Noktaları Dışarıda Bırakan Akıllı Kontur")
plt.axis('equal')
plt.show()
