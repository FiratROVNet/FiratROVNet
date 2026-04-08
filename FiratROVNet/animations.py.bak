"""
Animations Module
Görsel efektler, parçacık sistemleri ve patlama animasyonları.
"""

import random
from ursina import Entity, Vec3, color, lerp, time, destroy, curve


class GelismisParcacik(Entity):
    """
    Gelişmiş parçacık sistemi: Alev, enkaz, ışık hüzmeleri, duman, kıvılcım.
    """
    def __init__(self, pos, tur="kivilcim", renk=color.orange):
        super().__init__(
            model='sphere' if tur != 'enkaz' else 'cube',
            position=pos,
            add_to_scene_entities=True,
            double_sided=True
        )
        self.tur = tur
        self.timer = 0

        # --- FİZİK VE GÖRSEL AYARLAR ---
        if tur == "alev":
            self.model = 'sphere'
            # Başlangıçta daha küçük başla ki büyüdüğü belli olsun
            self.scale = Vec3(0.5, 0.5, 0.5) * random.uniform(0.8, 1.5)
            self.color = color.rgba(255, 255, 100, 1) # Parlak Sarı (Opak)
            self.hiz = random.uniform(2, 6)
            self.surtunme = 0.5
            self.omur = random.uniform(1.2, 1.8) # Ömrü uzattık
            self.yercekimi = 3.0
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.unlit = True 
            self.billboard = True 

        elif tur == "enkaz":
            self.model = 'cube'
            # MOLOZLAR DEVASA VE ASİMETRİK
            # x, y, z scale değerleri farklı olsun ki yamuk parçalar oluşsun
            sx = random.uniform(0.5, 1.5)
            sy = random.uniform(0.5, 1.5)
            sz = random.uniform(0.5, 1.5)
            self.scale = Vec3(sx, sy, sz) 
            self.color = renk 
            self.hiz = random.uniform(8, 25) # Daha uzağa fırlasınlar
            self.surtunme = 0.8 # Suda daha az yavaşlasınlar
            self.omur = random.uniform(3.0, 5.0) # Ekranda daha uzun kalsın
            self.yercekimi = -15 # Daha ağır oldukları için hızlı düşsünler
            self.rotation_vel = Vec3(random.uniform(-700,700), random.uniform(-700,700), random.uniform(-700,700))
            self.direction = Vec3(random.uniform(-1,1), random.uniform(0,1), random.uniform(-1,1)).normalized()

        elif tur == "huzme": 
            self.model = 'cube'
            self.scale = Vec3(0.05, 0.05, 1.5) 
            self.color = color.rgba(255, 255, 240, 1) 
            self.hiz = random.uniform(30, 60)
            self.surtunme = 3.0
            self.omur = random.uniform(0.2, 0.4)
            self.yercekimi = 0
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.look_at(self.position + self.direction) 
            self.unlit = True

        elif tur == "duman": 
            self.model = 'sphere'
            self.scale = Vec3(2.0, 2.0, 2.0) * random.uniform(0.8, 1.5)
            self.color = color.rgba(40, 40, 40, 0.8) 
            self.hiz = random.uniform(1, 5)
            self.surtunme = 1.0
            self.omur = random.uniform(2.0, 4.0)
            self.yercekimi = 4 
            self.direction = Vec3(random.uniform(-1,1), random.uniform(0, 1), random.uniform(-1,1)).normalized()
            self.billboard = True 

        elif tur == "kivilcim":
            self.scale = Vec3(0.2, 0.2, 0.2) 
            self.color = color.rgba(255, 200, 50, 1) 
            self.hiz = random.uniform(15, 35)
            self.surtunme = 2.5
            self.omur = random.uniform(0.5, 1.0)
            self.yercekimi = -5 
            self.direction = Vec3(random.uniform(-1,1), random.uniform(-1,1), random.uniform(-1,1)).normalized()
            self.unlit = True

    def update(self):
        dt = time.dt
        self.timer += dt
        ratio = self.timer / self.omur 

        # --- HAREKET ---
        self.position += self.direction * self.hiz * dt
        self.position.y += self.yercekimi * dt
        self.hiz = lerp(self.hiz, 0, dt * self.surtunme)

        # --- GÖRSEL DEĞİŞİM ---
        
        if self.tur == "alev":
            # BÜYÜME: Çok hızlı genişle
            buyume = dt * 12.0 
            self.scale += Vec3(buyume, buyume, buyume)
            
            # RENK GEÇİŞİ: Daha canlı ve belirgin
            if ratio < 0.3:
                # %0-30: Parlak Sarıdan Yoğun Turuncuya (Hala çok opak)
                self.color = lerp(
                    color.rgba(255, 255, 50, 1.0), 
                    color.rgba(255, 100, 0, 0.9), 
                    ratio * 3.3
                )
            elif ratio < 0.7:
                # %30-70: Turuncudan Kan Kırmızısına
                norm_ratio = (ratio - 0.3) * 2.5
                self.color = lerp(
                    color.rgba(255, 100, 0, 0.9), 
                    color.rgba(180, 20, 0, 0.6), 
                    norm_ratio
                )
            else:
                # %70-100: Kırmızıdan Siyah Dumana
                norm_ratio = (ratio - 0.7) * 3.3
                self.color = lerp(
                    color.rgba(180, 20, 0, 0.6), 
                    color.rgba(0, 0, 0, 0.0), 
                    norm_ratio
                )

        elif self.tur == "huzme":
            self.scale_z = lerp(self.scale_z, 0, dt * 8)
            self.alpha = lerp(1, 0, ratio)

        elif self.tur == "duman":
            self.scale += Vec3(dt*3, dt*3, dt*3)
            self.alpha = lerp(0.7, 0, ratio) 
        
        elif self.tur == "kivilcim":
            self.scale *= (1 - dt * 1.5)
            self.alpha = lerp(1, 0, ratio)

        elif self.tur == "enkaz":
            self.rotation += self.rotation_vel * dt 
            self.alpha = lerp(1, 0, ratio * ratio * ratio) # Son ana kadar görünür kalsın

        if self.timer > self.omur or self.scale_x < 0.01 or self.alpha <= 0.01:
            destroy(self)


def entity_patlat(hedef_entity, parca_sayisi=60, filo_ref=None):
    """
    Daha büyük, daha vahşi ve moloz dolu patlama efekti.
    
    Args:
        hedef_entity: Patlatılacak entity (genellikle ROV)
        parca_sayisi: Oluşturulacak parçacık sayısı
        filo_ref: Filo referansı (veri temizliği için)
    """
    if not hedef_entity or not hasattr(hedef_entity, 'world_position'): 
        return

    # --- YENİ EKLEME: Verileri anında temizle ---
    rov_id = getattr(hedef_entity, 'id', None)
    if rov_id is not None and filo_ref is not None:
        if hasattr(filo_ref, 'rov_verilerini_temizle'):
            filo_ref.rov_verilerini_temizle(rov_id)
    # -----------------------------------------

    pos = hedef_entity.world_position
    renk = getattr(hedef_entity, 'color', color.orange)
    
    # 1. MERKEZ PARLAMASI (Flash)
    flash = Entity(model='sphere', position=pos, scale=1.0, color=color.white, unlit=True, add_to_scene_entities=True)
    flash.animate_scale(25, duration=0.15, curve=curve.out_expo)
    flash.animate_color(color.rgba(255, 220, 100, 0), duration=0.3)
    destroy(flash, delay=0.35)

    # 2. ŞOK DALGASI
    shock = Entity(model='quad', texture='circle', position=pos, scale=1, rotation_x=90, 
                color=color.rgba(255, 255, 255, 0.6), unlit=True, add_to_scene_entities=True)
    shock.animate_scale(30, duration=0.4, curve=curve.out_quad)
    shock.animate_color(color.clear, duration=0.4)
    destroy(shock, delay=0.45)

    # 3. ALEV TOPLARI (Daha yoğun)
    # parca_sayisi * 0.6 kadar alev çıkacak (Eskiden 0.4 idi)
    for _ in range(int(parca_sayisi * 0.6)):
        GelismisParcacik(pos=pos, tur="alev")

    # 4. ENKAZ (MOLOZ) - SAYI ARTTIRILDI
    # Sabit 25-30 adet büyük moloz fırlat
    for _ in range(25):
        GelismisParcacik(pos=pos, tur="enkaz", renk=renk)

    # 5. IŞIK HÜZMELERİ
    for _ in range(int(parca_sayisi * 0.3)):
        GelismisParcacik(pos=pos, tur="huzme")

    # 6. DUMAN
    for _ in range(int(parca_sayisi * 0.4)):
        GelismisParcacik(pos=pos, tur="duman")

    # 7. SİLME İŞLEMİ
    if hasattr(hedef_entity, 'cikar'):
        print(f"🔥 ROV-{getattr(hedef_entity, 'id', '?')} havaya uçtu!")
        hedef_entity.cikar()
    else:
        destroy(hedef_entity)
