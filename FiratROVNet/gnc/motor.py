import math
import numpy as np
from ursina import Vec3, Entity, color
from ursina.models.procedural.cylinder import Cylinder
from panda3d.bullet import BulletRigidBodyNode
from FiratROVNet.config import Hidrodinamik

def _euler_deg_to_direction(rot_deg: Vec3):
    """
    REFERANS: schema_export.py içerisindeki mantıkla birebir aynıdır.
    Rotasyonlar tam olarak (X -> Z -> Y) sırasıyla matris çarpımıyla uygulanır.
    (0,0,0) -> (0,1,0) (dikey yukarı).
    """
    # Dereceleri radyana çevir
    rx, ry, rz = map(math.radians, [rot_deg.x, rot_deg.y, rot_deg.z])
    
    # Başlangıç: Motor dik duruyor (Azure durumu)
    v = np.array([0, 1, 0])

    # 1. X ekseninde yatır (Pitch) - rx: 90 ileri (+Z), -90 geri (-Z) yapar.
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(rx), -math.sin(rx)],
        [0, math.sin(rx), math.cos(rx)]
    ])
    
    # 2. Z ekseninde döndür (Roll/Açı) - Bu matris çizimdeki 'Ry' isimlendirmesine karşılık gelir
    # rz açısını kullanarak Y ekseni etrafındaki dönüşü simüle eder
    Ry_custom = np.array([
        [math.cos(rz), 0, math.sin(rz)],
        [0, 1, 0],
        [-math.sin(rz), 0, math.cos(rz)]
    ])
    
    # 3. Y ekseninde döndür (Yaw) - Bu matris çizimdeki 'Rz' isimlendirmesine karşılık gelir
    # ry açısını kullanarak Z ekseni etrafındaki dönüşü simüle eder
    Rz_custom = np.array([
        [math.cos(ry), -math.sin(ry), 0],
        [math.sin(ry), math.cos(ry), 0],
        [0, 0, 1]
    ])

    # Dönüşüm Zinciri (Schema dosyasındaki ile birebir aynı sıra):
    # res = Ry @ (Rz @ (Rx @ v))
    res = Ry_custom @ (Rz_custom @ (Rx @ v))
    
    return Vec3(res[0], res[1], res[2])


class Motor:
    def __init__(self, rov_entity: Entity):
        """
        :param rov_entity: ROV'un ana Ursina Entity nesnesi
        """
        self.yon_vec = None  
        self.rov_entity = rov_entity
        self.motor_entity = None  
        self.l_pos = None         
        self.physics_node = None 

    def ekle(self, koordinat: Vec3 = Vec3(0,0,0), yon_vec=(0,0,0)):
        """
        Motorun görselini ve fiziksel kuvvet uygulama noktasını ayarlar.
        """
        # Tuple veya Vec3 girişini normalize et
        rot_deg = Vec3(yon_vec[0] if hasattr(yon_vec, '__getitem__') else yon_vec.x,
                       yon_vec[1] if hasattr(yon_vec, '__getitem__') else yon_vec.y,
                       yon_vec[2] if hasattr(yon_vec, '__getitem__') else yon_vec.z)

        # FİZİK: yon_vec artık Schema çizimindeki okların yönüyle aynı matematiksel sonucu verir
        self.yon_vec = _euler_deg_to_direction(rot_deg)

        # GÖRSEL RENK: Azure (Dikey) vs Green (Yatay)
        # Ry (Yaw) 0 ise ve X/Z eğikse dikey kabul edilir
        motor_color = color.azure if abs(rot_deg.y) == 0 and (abs(rot_deg.x) > 5 or abs(rot_deg.z) > 5) else color.green
        
        self.motor_entity = Entity(
            model=Cylinder(resolution=12, radius=1.0, height=1.0),
            color=motor_color,
            position=koordinat,
            parent=self.rov_entity,
            unlit=True
        )

        # Ölçekleme
        sx = getattr(self.rov_entity, 'scale_x', 1.0) or 1.0
        sy = getattr(self.rov_entity, 'scale_y', 1.0) or 1.0
        sz = getattr(self.rov_entity, 'scale_z', 1.0) or 1.0
        self.motor_entity.scale = Vec3(0.5 / sx, 0.5 / sy, 0.5 / sz)

        # GÖRSEL ROTASYON: Ursina rotasyonu çizim parametreleriyle aynı kalır
        self.motor_entity.rotation = Vec3(rot_deg.x, rot_deg.y, rot_deg.z)

        self.l_pos = koordinat
        self._find_physics_node()

    def _find_physics_node(self):
        """Fizik düğümünü hiyerarşide bulur."""
        if hasattr(self.rov_entity, 'physics_node') and isinstance(self.rov_entity.physics_node, BulletRigidBodyNode):
            self.physics_node = self.rov_entity.physics_node
        elif hasattr(self.rov_entity, 'node') and isinstance(self.rov_entity.node, BulletRigidBodyNode):
            self.physics_node = self.rov_entity.node
        else:
            p = getattr(self.rov_entity, 'parent', None)
            while p is not None:
                if hasattr(p, 'physics_node'):
                    self.physics_node = p.physics_node
                    break
                p = getattr(p, 'parent', None)

    def calistir(self, guc: float):
            """
            Motoru çalıştırır. Verilen güce göre fizik motoruna sürekli kuvvet uygular.
            """
            # Eğer guc 0 ise veya çok küçükse fizik motorunu boşuna yorma
            if abs(guc) < 0.001:
                return

            if self.physics_node is not None and self.l_pos is not None:
                self.physics_node.setActive(True)

                # 1. Motorun itki yönünü, ROV'un o anki dünya yönelimine göre çevir
                world_force_vec = self.rov_entity.quaternion.xform(self.yon_vec)
                
                # 2. GERÇEK MOMENT KOLU (DİKKAT: .normalized() SİLİNDİ!)
                # Motorun ROV üzerindeki yerel pozisyonunu, ROV'un güncel boyutuyla (scale) çarpıyoruz.
                # Bu, motorun merkeze olan GERÇEK fiziksel uzaklığını verir.
                actual_l_pos = Vec3(
                    self.l_pos.x * self.rov_entity.scale_x,
                    self.l_pos.y * self.rov_entity.scale_y,
                    self.l_pos.z * self.rov_entity.scale_z
                )
                
                # 3. Merkeze olan bu uzaklık vektörünü, ROV'un dönüşüne göre dünya eksenine uyarla
                # Bullet, kuvvetin uygulanacağı noktayı "dünya ekseninde merkezden uzaklık" olarak bekler.
                world_rel_pos = self.rov_entity.quaternion.xform(actual_l_pos)

                # 4. Uygulanacak nihai kuvvet büyüklüğü (Newton)
                final_force = world_force_vec * float(guc) * Hidrodinamik.MAX_ITME_KUVVETI

                # 5. Bullet Physics'e kuvveti ilet
                # Bu fonksiyon inanılmaz güçlüdür: 'world_rel_pos' merkezden farklı olduğu için
                # Bullet hem aracı iter (Force) hem de otomatik olarak döndürür (Torque).
                self.physics_node.applyForce(final_force, world_rel_pos)