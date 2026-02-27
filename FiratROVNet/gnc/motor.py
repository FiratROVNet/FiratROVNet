from ursina import Vec3, Entity, color
from ursina.models.procedural.cylinder import Cylinder
from panda3d.bullet import BulletRigidBodyNode

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

    def ekle(self, koordinat: Vec3 = Vec3(0,0,0), yon_vec: tuple = (0,0,1)):
        """
        Motorun görselini ve fiziksel kuvvet uygulama noktasını ayarlar.
        :param koordinat: Motorun ROV üzerindeki yerel pozisyonu (Vec3)
        :param yon_vec: Motorun itiş yönü (Varsayılan 0,0,1 ileri)
        """
        # Yön vektörünü normalize ederek (birim vektör) kaydediyoruz
        self.yon_vec = Vec3(*yon_vec).normalized()

        # Görsel Silindiri Oluştur
        self.motor_entity = Entity(
            model=Cylinder(resolution=12, radius=0.1, height=0.4),
            color=color.azure,
            position=koordinat,
            parent=self.rov_entity
        )
        
        # Silindiri itiş yönüne doğru çevir (Görsel olarak mantıklı durması için)
        self.motor_entity.look_at(self.motor_entity.world_position + self.yon_vec)

        self.l_pos = koordinat

        # Physics node'u güvenli bir şekilde bul
        if hasattr(self.rov_entity, 'physics_node') and isinstance(self.rov_entity.physics_node, BulletRigidBodyNode):
            self.physics_node = self.rov_entity.physics_node
        elif hasattr(self.rov_entity, 'node') and isinstance(self.rov_entity.node, BulletRigidBodyNode):
            self.physics_node = self.rov_entity.node
        else:
            self.physics_node = None
            parent = getattr(self.rov_entity, 'parent', None)
            while parent is not None:
                if hasattr(parent, 'physics_node') and isinstance(parent.physics_node, BulletRigidBodyNode):
                    self.physics_node = parent.physics_node
                    break
                parent = getattr(parent, 'parent', None)
                
        if self.physics_node is None:
            print(f'[Motor] Uyarı: {self.rov_entity.id} idli ROV için Physics node bulunamadı.')

    def calistir(self, guc: float):
        """
        Motoru çalıştırır ve Bullet Physics üzerinden kuvvet uygular.
        """
        if self.physics_node is not None and self.l_pos is not None:
            # 1. Node'u uyandır (Uyku modundaysa kuvvet etki etmez)
            self.physics_node.setActive(True)

            # 2. Lokal itiş yönünü, ROV'un güncel dönüşüne (Quaternion) göre Dünya Yönüne çevir
            world_force_vec = self.rov_entity.quaternion.xform(self.yon_vec)
            
            # 3. Lokal motor pozisyonunu, ROV'un güncel dönüşüne göre Dünya Pozisyonuna çevir
            # DİKKAT: Dönüşlerdeki kilitlenmeyi (90 derecede kalma) bu satır çözer!
            world_rel_pos = self.rov_entity.quaternion.xform(self.l_pos)

            # 4. Kuvveti Hesapla (Kütle 15kg olduğu için yüksek bir çarpan gerekiyor)
            FORCE_MULT = 15000.0  
            final_force = world_force_vec * float(guc) * FORCE_MULT

            # 5. Kuvveti Uygula -> applyForce(Kuvvet Vektörü, Uygulama Noktası)
            self.physics_node.applyForce(final_force, world_rel_pos)
        else:
            pass # Fiziği olmayan entityler (örneğin patlamış ROV'lar) için hata basma