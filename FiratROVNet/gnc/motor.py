import math
import numpy as np
from ursina import Vec3, Entity, color
from ursina.models.procedural.cylinder import Cylinder
from panda3d.bullet import BulletRigidBodyNode
from panda3d.core import Vec3 as P3Vec
from FiratROVNet.config import Hidrodinamik


class Motor:
    def __init__(self, rov_entity: Entity, filo_ref=None):
        """
        :param rov_entity: ROV'un ana Ursina Entity nesnesi
        :param filo_ref: Filo referansı; _euler_deg_to_direction bu üzerinden çağrılır (opsiyonel)
        """
        self.yon_vec = None
        self.rov_entity = rov_entity
        self.filo_ref = filo_ref
        self.motor_entity = None
        self.l_pos = None
        self.physics_node = None


    def ekle(self, koordinat: Vec3 = Vec3(0,0,0), yon_vec=(0,0,0)):
        """
        Motorun görselini ve fiziksel itki parametrelerini ayarlar.
        
        Özellikler:
        - Silindir başlangıçta (0,0,0 rotasyonda) Z eksenine paraleldir.
        - Origin noktası silindirin tam tabanıdır.
        - Ölçekleme, ROV ölçeğinden bağımsız olarak 0.5 birim sabit boy sağlar.
        """
        # 1. Giriş rotasyonunu Vec3 formatına getir
        rot_deg = Vec3(yon_vec) if isinstance(yon_vec, (list, tuple, Vec3)) else Vec3(0,0,0)

        # 2. FİZİKSEL YÖN HESABI
        # Başlangıçta silindiri Z eksenine paralel kabul ettiğimiz için v=(0,0,1)
        filo = self.filo_ref
        if filo is not None:
            self.yon_vec = filo._euler_deg_to_direction(rot_deg, v=Vec3(0, 1, 0))
        else:
            self.yon_vec = Vec3(0, 1, 0)

        # 3. RENK MANTIĞI
        is_vertical = abs(rot_deg.y) == 0 and (abs(rot_deg.x) > 5 or abs(rot_deg.z) > 5)
        motor_color = color.azure if is_vertical else color.green

        # 4. PIVOT ENTITY (Ana Taşıyıcı ve Rotasyon Merkezi)
        self.motor_entity = Entity(
            parent=self.rov_entity,
            position=koordinat,
            rotation=rot_deg
        )

        # 5. ÖLÇEKLENDİRME (İstediğiniz özel yapı)
        # ROV'un (parent) ölçeğini alarak motoru 0.5 birim sabit boya sabitler
        sx = getattr(self.rov_entity, 'scale_x', 1.0) or 1.0
        sy = getattr(self.rov_entity, 'scale_y', 1.0) or 1.0
        sz = getattr(self.rov_entity, 'scale_z', 1.0) or 1.0
        self.motor_entity.scale = Vec3(0.5 / sx, 0.5 / sy, 0.5 / sz)

        # 6. VISUAL ENTITY (Silindir Modeli)
        # Pivot'un çocuğu olduğu için üstteki ölçekleme ve rotasyondan etkilenir
        self.visual_entity = Entity(
            model=Cylinder(resolution=12, radius=1.0, height=1),
            parent=self.motor_entity,
            color=motor_color,
            rotation=(0, 0, 0), 
            unlit=True
        )

        # Teknik verileri kaydet
        self.l_pos = koordinat
        self._find_physics_node()

    @property
    def color(self):
        """Görsel silindirin rengi (motor_entity.color)."""
        if self.motor_entity is not None:
            return self.motor_entity.color
        return getattr(self, "_color", color.white)

    @color.setter
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

    def calistir(self, guc: float):
            if abs(guc) < 0.001 or math.isnan(guc):
                return

            if self.physics_node is not None and self.l_pos is not None:
                self.physics_node.setActive(True)
                quat = self.rov_entity.quaternion

                # 1. KUVVET BÜYÜKLÜĞÜ
                mag = float(guc) * Hidrodinamik.MAX_ITME_KUVVETI
                
                # 2. DÜNYA EKSENİNDE ÇİZGİSEL KUVVET (İTME)
                world_force = quat.xform(self.yon_vec) * mag
                
                # 3. GERÇEK MOMENT KOLU
                actual_l_pos = Vec3(
                    self.l_pos.x * self.rov_entity.scale_x,
                    self.l_pos.y * self.rov_entity.scale_y,
                    self.l_pos.z * self.rov_entity.scale_z
                )
                world_rel_pos = quat.xform(actual_l_pos)

                # 4. TORKU MANUEL HESAPLA (r x F)
                world_torque = world_rel_pos.cross(world_force)

                # 5. Y EKSENİ DÜZELTMESİ (KRİTİK NOKTA!)
                # Fizik motorunun Sağ El - Sol El çatışmasını gidermek için 
                # dönüş ekseninin (Yaw) yönünü tersine çeviriyoruz.
                world_torque.y = -world_torque.y

                # 6. FİZİK MOTORUNA İLET (applyForce Yerine Ayrı Ayrı)
                if not world_force.is_nan():
                    # Aracı ötelemek için merkeze itki uygula
                    self.physics_node.applyCentralForce(P3Vec(world_force.x, world_force.y, world_force.z))
                
                if not world_torque.is_nan():
                    # Aracı döndürmek için düzeltilmiş torku uygula
                    self.physics_node.applyTorque(P3Vec(world_torque.x, world_torque.y, world_torque.z))