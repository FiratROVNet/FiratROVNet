import math
import numpy as np  # type: ignore[import]
from ursina import Vec3, Entity, color  # type: ignore[import]
from ursina.models.procedural.cylinder import Cylinder  # type: ignore[import]
from panda3d.bullet import BulletRigidBodyNode  # type: ignore[import]
from panda3d.core import Vec3 as P3Vec  # type: ignore[import]
from FiratROVNet.config import Hidrodinamik  # type: ignore[import]

class Motor:
    def __init__(self, rov_entity: Entity, filo_ref=None):
        """
        :param rov_entity: ROV'un ana Ursina Entity nesnesi
        :param filo_ref: Filo referansı; _euler_deg_to_direction bu üzerinden çağrılır (opsiyonel)
        """
        self.yon_vec: Vec3 | None = None
        self.rov_entity = rov_entity
        self.filo_ref = filo_ref
        self.motor_entity: Entity | None = None
        self.visual_entity: Entity | None = None
        self.l_pos_ursina: Vec3 | None = None  # Ursina biriminde (görsel için)
        self.metre_pos: Vec3 | None = None     # Metre cinsinden (fizik için!)
        self.physics_node = None
        self._color = color.white  # Renk cache (motor_entity yokken)
        self.tork_bv = Vec3(0, 0, 0)  # Birim tork vektörü
        self.r_bv = Vec3(0, 0, 0)     # Birim itki vektörü
        self.is_vertical = False

    def ekle(self, koordinat_metre: Vec3 = Vec3(0,0,0), yon_vec=(0,0,0)):
        """
        Motoru metre cinsinden konumlandır!
        koordinat_metre: ROV merkezine göre METRE cinsinden konum (örnek: (1.8, 0, 1.8))
        yon_vec: Motorun yönü (pitch, yaw, roll) derece cinsinden
        """
        # 1. Metre cinsinden konumu kaydet (fizik için)
        self.metre_pos = Vec3(koordinat_metre) if isinstance(koordinat_metre, Vec3) else Vec3(*koordinat_metre)
        
        # 2. Giriş rotasyonunu Vec3 formatına getir
        rot_deg = Vec3(yon_vec) if isinstance(yon_vec, (list, tuple, Vec3)) else Vec3(0,0,0)

        # 3. FİZİKSEL YÖN HESABI
        # Ursinalı özel "sol elli" mantıktan çıkmaması ve tork vektörlerinin doğru yönlere çalışabilmesi için Orjinal _euler_deg_to_direction metodu kullanılmalı. 
        filo = self.filo_ref
        if filo is not None:
            self.yon_vec = filo._euler_deg_to_direction(rot_deg, v=Vec3(0, 1, 0))
        else:
            self.yon_vec = Vec3(0, 1, 0)

        # Birim vektör olduğundan emin ol
        if self.yon_vec.length() > 0.001:  # type: ignore[union-attr]
            self.yon_vec = self.yon_vec.normalized()  # type: ignore[union-attr]
        
        self.r_bv = self.yon_vec  # Birim itki vektörü

        # 4. GÖRSEL İÇİN: Metre'yi Ursina birimine çevir, ROV scale'ine BÖL!
        # Kullanıcı 1.8 metre gönderdi → ROV scale 0.009 ise → 1.8 / 0.009 = 200 birim
        scale_x = getattr(self.rov_entity, 'scale_x', 1.0) or 1.0
        scale_y = getattr(self.rov_entity, 'scale_y', 1.0) or 1.0
        scale_z = getattr(self.rov_entity, 'scale_z', 1.0) or 1.0
        
        self.l_pos_ursina = Vec3(
            self.metre_pos.x / scale_x,  # type: ignore[union-attr]  # Metre / scale = Ursina birimi
            self.metre_pos.y / scale_y,  # type: ignore[union-attr]
            self.metre_pos.z / scale_z   # type: ignore[union-attr]
        )

        # 5. RENK MANTIĞI
        self.is_vertical = abs(rot_deg.y) == 0 and (abs(rot_deg.x) > 5 or abs(rot_deg.z) > 5)
        motor_color = color.azure if self.is_vertical else color.green

        # 6. PIVOT ENTITY (Ana Taşıyıcı ve Rotasyon Merkezi)
        self.motor_entity = Entity(
            parent=self.rov_entity,
            position=self.l_pos_ursina,  # Ursina biriminde!
            rotation=rot_deg
        )

        # 7. ÖLÇEKLENDİRME (Görsel boyut - ROV ölçeğinden bağımsız)
        # Motor görseli sabit 0.5 birim boyunda olsun
        self.motor_entity.scale = Vec3(0.5 / scale_x, 0.5 / scale_y, 0.5 / scale_z)  # type: ignore[union-attr]

        # 8. VISUAL ENTITY (Silindir Modeli)
        self.visual_entity = Entity(
            model=Cylinder(resolution=12, radius=1.0, height=1),
            parent=self.motor_entity,
            color=motor_color,
            rotation=(0, 0, 0), 
            unlit=True
        )

        # 9. Fizik düğümünü bul
        self._find_physics_node()

    @property
    def color(self):
        """Görsel silindirin rengi (motor_entity.color)."""
        if self.motor_entity is not None:
            return self.motor_entity.color
        return getattr(self, "_color", color.white)

    @color.setter  # type: ignore[attr-defined]
    def color(self, value):
        if self.motor_entity is not None:
            self.motor_entity.color = value
        else:
            self._color = value

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

    def _vec3_gecerli_mi(self, vec) -> bool:
        """Vektörün geçerli (finite) olup olmadığını kontrol eder"""
        try:
            return (math.isfinite(float(vec.x)) and 
                    math.isfinite(float(vec.y)) and 
                    math.isfinite(float(vec.z)) and
                    abs(vec.x) < 1e6 and abs(vec.y) < 1e6 and abs(vec.z) < 1e6)
        except (AttributeError, TypeError, ValueError):
            return False

    def calistir(self, guc: float):
            """
            Motoru çalıştır - applyForce ile GELİŞMİŞ ve FİZİKSEL olarak en doğru yöntem.
            """
            if abs(guc) < 0.001 or math.isnan(guc) or math.isinf(guc):
                return
            
            guc = max(-1.0, min(1.0, float(guc)))

            if self.physics_node is None or self.metre_pos is None:
                return
                
            self.physics_node.setActive(True)  # type: ignore[union-attr]
            
            if self.yon_vec is None or self.yon_vec.length() <= 1e-6:  # type: ignore[union-attr]
                return

            # ==========================================
            # 1. PANDA3D DÜNYA MATRİSİNİ AL (Bullet'in sağ-elli + y-up-left dönüşümü için Kritik!)
            # ==========================================
            # Quat() kullanımında Panda3D'nin sol-elli y-up'a uyumlamak için attığı Transform/Scale
            # (-1 gibi ayna katsayıları) kaybolur! O yüzden matris tabanlı xformVec KULLANILMALIDIR!
            rov_mat = getattr(self.rov_entity, '_world_mat', None)
            if rov_mat is None:
                rov_mat = self.rov_entity.physics_np.getNetTransform().getMat()

            # ==========================================
            # 2. KUVVET YÖNÜNÜ DÜNYA KOORDİNATINA ÇEVİR
            # ==========================================
            local_force_dir = P3Vec(self.yon_vec.x, self.yon_vec.y, self.yon_vec.z)  # type: ignore[union-attr]
            
            # xformVec: Matris ile ölçeklendirme/yansıtma dahil vektör/açı hesaplar
            world_force_dir = rov_mat.xformVec(local_force_dir)
            world_force_dir.normalize()
            
            # Kuvvetin Newton cinsinden büyüklüğü
            force_magnitude = float(guc) * Hidrodinamik.MAX_ITME_KUVVETI
            world_force = world_force_dir * force_magnitude

            # ==========================================
            # 3. UYGULAMA NOKTASINI (OFFSET) HESAPLA -> KRİTİK DÜZELTME!
            # ==========================================
            local_pos = P3Vec(self.metre_pos.x, self.metre_pos.y, self.metre_pos.z)  # type: ignore[union-attr]
            
            # DİKKAT: applyForce, pozisyon olarak DÜNYA KONUMUNU DEĞİL, 
            # Ağırlık Merkezine (CoM) olan MESAFEYİ (Offset) bekler!
            # Quat() yerine xformVec kullanıyoruz ki sağ/sol el uyumsuzluğu Scale -1 ile absorbe edilsin.
            world_offset = rov_mat.xformVec(local_pos)
            

            # ==========================================
            # 4. FİZİK MOTORUNA UYGULA
            # ==========================================
            if (math.isfinite(world_force.x) and math.isfinite(world_force.y) and math.isfinite(world_force.z) and
                math.isfinite(world_offset.x) and math.isfinite(world_offset.y) and math.isfinite(world_offset.z)):
                
                # Parametreler: (Dünya ekseninde Kuvvet, Dünya ekseninde CoM'den offset)
                # Tork fizik motoru tarafından (world_offset x world_force) şeklinde OTOMATİK hesaplanır!
                self.physics_node.applyForce(world_force, world_offset)  # type: ignore[union-attr]
                
    def debug_bilgi(self):
        """Motor hakkında debug bilgisi döndürür"""
        def vec3_to_tuple(vec):
            if vec is None:
                return None
            try:
                return (float(vec.x), float(vec.y), float(vec.z))
            except (AttributeError, TypeError, ValueError):
                return None

        return {
            'yon_vec': vec3_to_tuple(self.yon_vec),
            'metre_pos': vec3_to_tuple(self.metre_pos),
            'l_pos_ursina': vec3_to_tuple(self.l_pos_ursina),
            'is_vertical': self.is_vertical,
            'tork_bv': vec3_to_tuple(self.tork_bv),
            'r_bv': vec3_to_tuple(self.r_bv)
        }