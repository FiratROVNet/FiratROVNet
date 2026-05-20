from ursina import Vec3

from FiratROVNet.gnc.motor import Motor


class MotorDuzeni:
    def __init__(self, filo_ref):
        self.filo = filo_ref

    def BlueROV2_motor_konfigurasyonu(self, rov):
        rov.m0 = Motor(rov, filo_ref=self.filo)
        rov.m0.ekle(koordinat_metre=Vec3(-2, 0.0, 2), yon_vec=(90, 45, 0))
        rov.m0.name = "m0"
        self.filo.motorlar[rov.id].append(rov.m0)

        rov.m1 = Motor(rov, filo_ref=self.filo)
        rov.m1.ekle(koordinat_metre=Vec3(2, 0.0, 2), yon_vec=(90, -45, 0))
        rov.m1.name = "m1"
        self.filo.motorlar[rov.id].append(rov.m1)

        rov.m2 = Motor(rov, filo_ref=self.filo)
        rov.m2.ekle(koordinat_metre=Vec3(-2, 0.0, -2), yon_vec=(90, 135, 0))
        rov.m2.name = "m2"
        self.filo.motorlar[rov.id].append(rov.m2)

        rov.m3 = Motor(rov, filo_ref=self.filo)
        rov.m3.ekle(koordinat_metre=Vec3(2, 0.0, -2), yon_vec=(90, -135, 0))
        rov.m3.name = "m3"
        self.filo.motorlar[rov.id].append(rov.m3)

        rov.m4 = Motor(rov, filo_ref=self.filo)
        rov.m4.ekle(koordinat_metre=Vec3(-1.0, 0.0, 0.6), yon_vec=(0.0, 0, 0.0))
        rov.m4.name = "m4"
        self.filo.motorlar[rov.id].append(rov.m4)

        rov.m5 = Motor(rov, filo_ref=self.filo)
        rov.m5.ekle(koordinat_metre=Vec3(1.0, 0.0, 0.6), yon_vec=(0.0, 0, 0.0))
        rov.m5.name = "m5"
        self.filo.motorlar[rov.id].append(rov.m5)

        rov.m6 = Motor(rov, filo_ref=self.filo)
        rov.m6.ekle(koordinat_metre=Vec3(-1.0, 0.0, -0.6), yon_vec=(0.0, 0, 0.0))
        rov.m6.name = "m6"
        self.filo.motorlar[rov.id].append(rov.m6)

        rov.m7 = Motor(rov, filo_ref=self.filo)
        rov.m7.ekle(koordinat_metre=Vec3(1.0, 0.0, -0.6), yon_vec=(0.0, 0, 0.0))
        rov.m7.name = "m7"
        self.filo.motorlar[rov.id].append(rov.m7)

    def tum_motor_bv_kutuphanelerini_guncelle(self):
        self.filo.motorlar_bv = {}
        for rov_id, motor_listesi in self.filo.motorlar.items():
            rov = self.filo.find_rov_by_id(rov_id)
            if not rov:
                continue

            rov_icin_bv_listesi = []
            for motor in motor_listesi:
                if getattr(motor, "motor_entity", None):
                    rot = motor.motor_entity.rotation
                    birim_vektor = self.filo._euler_deg_to_direction(Vec3(rot.x, rot.y, rot.z))
                else:
                    birim_vektor = getattr(motor, "r_bv", Vec3(0, 1, 0))
                motor.r_bv = birim_vektor
                rov_icin_bv_listesi.append(birim_vektor)

                tork_vec = motor.metre_pos.cross(birim_vektor)
                if tork_vec.is_nan() or tork_vec.length() <= 1e-6:
                    motor.tork_bv = Vec3(0, 0, 0)
                else:
                    motor.tork_bv = tork_vec.normalized()

            self.filo.motorlar_bv[rov_id] = rov_icin_bv_listesi
            rov.motorlar = motor_listesi
