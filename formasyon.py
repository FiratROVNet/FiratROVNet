from FiratROVNet.simulasyon import Ortam
from FiratROVNet.gnc import Filo
from FiratROVNet.config import cfg
from ursina import *
import numpy as np
import heapq
import os
import random

# =============================================================================
# 1. A* PLANLAYICI SINIFI (Senin Mantığın)
# =============================================================================
class AStarPlanlayici:
    def __init__(self, width, height, safety_padding=15): 
        # Havuz -100 ile +100 arasında (toplam 200 birim)
        self.width = int(width)   # 200
        self.height = int(height) # 200
        self.safety_padding = safety_padding
        self.grid = np.zeros((self.width, self.height))

    def harita_guncelle(self, ursina_engeller):
        """Ursina engellerini A* gridine işler"""
        self.grid = np.zeros((self.width, self.height))
        
        for engel in ursina_engeller:
            # Ursina (-100, 100) -> Grid (0, 200) dönüşümü
            ox = int(engel.x + 100)
            oy = int(engel.z + 100) # Ursina'da Z ekseni, 2D haritada Y eksenidir
            
            x_min = max(0, ox - self.safety_padding)
            x_max = min(self.width, ox + self.safety_padding)
            y_min = max(0, oy - self.safety_padding)
            y_max = min(self.height, oy + self.safety_padding)
            self.grid[x_min:x_max, y_min:y_max] = 1

    def heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

    def planla(self, start_pos, goal_pos):
        # Ursina Vec3 -> Grid (x, y) dönüşümü
        start = (int(start_pos.x + 100), int(start_pos.z + 100))
        goal = (int(goal_pos.x + 100), int(goal_pos.z + 100))
        
        # Sınır kontrolü
        start = (np.clip(start[0], 0, self.width-1), np.clip(start[1], 0, self.height-1))
        goal = (np.clip(goal[0], 0, self.width-1), np.clip(goal[1], 0, self.height-1))

        if self.grid[start[0]][start[1]] == 1: 
            print("⚠️ Başlangıç noktası engel içinde!")
            return []

        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            if self.heuristic(current, goal) < 5.0:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                # Yolu ters çevir ve Grid -> Ursina koordinatına dönüştür
                path = []
                for p in data[::-1][::3]: # Yolu seyrelterek al
                    path.append(Vec3(p[0]-100, 0, p[1]-100))
                path.append(Vec3(goal[0]-100, 0, goal[1]-100))
                return path
            
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j 
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height:
                    if self.grid[neighbor[0]][neighbor[1]] == 1: continue
                else: continue
                
                tentative_g_score = gscore[current] + np.sqrt(i**2+j**2)
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0): continue
                
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [k[1] for k in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return []

# =============================================================================
# 2. ORTAM VE SİMÜLASYON KURULUMU (Örnek main.py yapısı)
# =============================================================================
print("🔵 Fırat-GNC + Dinamik Formasyon Sistemi Başlatılıyor...")
app = Ortam()

# Hedef
hedef_nokta = Vec3(40, 0, 60)

# Simülasyonu oluştur (Örnekteki gibi)
app.sim_olustur(
    n_rovs=4, 
    n_engels=25, 
    hedef_nokta=hedef_nokta, 
    rov_model_yolu="rov.obj" # Varsa model, yoksa küp
)

# Filo Kurulumu (ZORUNLU ADIM)
filo = Filo()
# ROV rollerini manuel olarak ayarla
app.rovs[0].set("rol", 1)  # ROV-0 lider
for i in range(1, len(app.rovs)):
    app.rovs[i].set("rol", 0)  # Diğerleri takipçi

filo.otomatik_kurulum(
    rovs=app.rovs,
    baslangic_hedefleri={
        0: (-20, 0, -20),  # Lider başlangıç noktası
        1: (-25, -5, -25), 
        2: (-15, -5, -25),
        3: (-20, -5, -30)
    },
    sensor_ayarlari={'engel_mesafesi': 30.0} # A* için geniş algılama
)
app.filo = filo

# Konsola ekle
app.konsola_ekle("git", filo.git)
app.konsola_ekle("gnc", filo.sistemler)
app.konsola_ekle("filo", filo)
app.konsola_ekle("rovs", app.rovs)

# =============================================================================
# 3. FORMASYON YÖNETİCİSİ (BEYİN)
# =============================================================================
class FormasyonYoneticisi:
    def __init__(self, app_ref, filo_ref, hedef):
        self.app = app_ref
        self.filo = filo_ref
        self.hedef = hedef
        
        # A* Başlat
        self.planner = AStarPlanlayici(200, 200, safety_padding=18)
        self.planner.harita_guncelle(self.app.engeller)
        
        # İlk rotayı hesapla
        self.path = self.planner.planla(self.app.rovs[0].position, self.hedef)
        self.path_index = 0
        self.formation_type = "V_SHAPE"

    def get_formation_offset(self, index):
        gap = 8.0 # Ara mesafe
        if self.formation_type == "V_SHAPE":
            row = (index + 1) // 2
            side = 1 if index % 2 != 0 else -1
            return Vec3(side * gap * row * 1.5, 0, -gap * row)
        else: # LINE
            return Vec3(0, 0, -gap * index)

    def update(self):
        # 1. Liderin durumu
        lider_pos = self.app.rovs[0].position
        
        # 2. Formasyon Değiştirme Kararı
        min_dist = 999
        for engel in self.app.engeller:
            dist = distance(lider_pos, engel.position)
            if dist < min_dist: min_dist = dist
            
        if min_dist < 35.0:
            self.formation_type = "LINE"
        else:
            self.formation_type = "V_SHAPE"

        # 3. Lideri A* Rotalarında Yürüt
        target_pos = lider_pos
        if self.path and self.path_index < len(self.path):
            waypoint = self.path[self.path_index]
            
            # Yüksekliği (Y) yok sayarak 2D mesafe kontrolü
            flat_pos = Vec3(lider_pos.x, 0, lider_pos.z)
            flat_wp = Vec3(waypoint.x, 0, waypoint.z)
            
            if distance(flat_pos, flat_wp) < 5.0:
                self.path_index += 1
            else:
                target_pos = waypoint
        else:
            target_pos = self.hedef

        # Lideri GNC sistemiyle o noktaya gönder
        self.filo.git(0, target_pos.x, target_pos.z, 0)
        
        # 4. Takipçileri Formasyona Sok
        lider_entity = self.app.rovs[0]
        for i in range(1, len(self.filo.sistemler)):
            local_offset = self.get_formation_offset(i)
            # Liderin yönüne göre ofseti döndür
            world_offset = (lider_entity.right * local_offset.x) + \
                           (lider_entity.forward * local_offset.z)
            takipci_hedef = lider_pos + world_offset
            
            self.filo.git(i, takipci_hedef.x, takipci_hedef.z, -10)
            
        # Debug Çizim (Rotayı yeşil küplerle göster)
        if hasattr(self, 'debug_timer'): self.debug_timer -= time.dt
        else: self.debug_timer = 0

        if self.path and self.path_index < len(self.path) and self.debug_timer <= 0:
            for p in self.path[self.path_index:]:
                if random.random() < 0.1:
                    e = Entity(model='cube', scale=0.5, color=color.green, position=p, unlit=True)
                    destroy(e, delay=0.1)
            self.debug_timer = 0.5

# Yöneticiyi başlat
yonetici = FormasyonYoneticisi(app, filo, hedef_nokta)
app.konsola_ekle("yonetici", yonetici)

# =============================================================================
# 4. ANA DÖNGÜ (UPDATE LOOP) - EN KRİTİK KISIM
# =============================================================================

def ana_dongu():
    """Her frame'de çalışacak fonksiyon"""
    # 1. Simülasyonun kendi fizik güncellemeleri (pil, çarpışma, hareket)
    app.guncelle()
    
    # 2. Senin Dinamik Formasyon Mantığın
    yonetici.update()

# Simülasyona bizim özel döngümüzü kullanmasını söylüyoruz
app.set_update_function(ana_dongu)

print("✅ Sistem aktif. A* ve Dinamik Formasyon devrede.")

# =============================================================================
# 5. ÇALIŞTIRMA (RUN)
# =============================================================================
if __name__ == "__main__":
    try: 
        # Simülasyonu başlatan sihirli komut
        app.run(interaktif=True)
    except KeyboardInterrupt: 
        pass
    finally: 
        try:
            os.system('stty sane')
        except:
            pass
        os._exit(0) 
        #Güncelleme 
            os.system('stty sane')
        except:
            pass
        os._exit(0) 
        #Güncelleme 