import math

from ursina import Vec3


class ModulYardimcisi:
    def __init__(self, filo_ref):
        self.filo = filo_ref

    def _euler_deg_to_direction(self, rot_deg: Vec3, v=Vec3(0, 1, 0)):
        rx = math.radians(rot_deg.x)
        ry = math.radians(rot_deg.y)
        rz = math.radians(-rot_deg.z)

        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)

        v1x = v.x * cz - v.y * sz
        v1y = v.x * sz + v.y * cz
        v1z = v.z

        v2x = v1x
        v2y = v1y * cx - v1z * sx
        v2z = v1y * sx + v1z * cx

        resx = v2x * cy + v2z * sy
        resy = v2y
        resz = -v2x * sy + v2z * cy

        return Vec3(resx, resy, resz)

    def dunya_to_yerel_vektor(self, dunya_vektor: Vec3, rotasyon: Vec3) -> Vec3:
        rov_ileri = self._euler_deg_to_direction(rotasyon, v=Vec3(0, 0, 1))
        rov_sag = self._euler_deg_to_direction(rotasyon, v=Vec3(1, 0, 0))
        rov_yukari = self._euler_deg_to_direction(rotasyon, v=Vec3(0, 1, 0))

        yerel_x = dunya_vektor.dot(rov_sag)
        yerel_y = dunya_vektor.dot(rov_yukari)
        yerel_z = dunya_vektor.dot(rov_ileri)
        return Vec3(yerel_x, yerel_y, yerel_z)

    def tum_motorlarin_guclerini_hesapla(
        self,
        rov_id=0,
        hedef_vektor_dunya: Vec3 = Vec3(0.0, 0.0, 0.0),
        guc: float = 0.0,
    ):
        rov = self.filo.find_rov_by_id(rov_id)
        if not rov:
            return [0.0] * 6

        motorlar = getattr(rov, "motorlar", [])
        cached_powers = [float(getattr(motor, "guc", 0.0)) for motor in motorlar]
        motor_bv = self.filo.motorlar_bv.get(rov_id, [])
        if not motor_bv:
            return cached_powers if cached_powers else [0.0] * 6

        hedef_yerel = self.dunya_to_yerel_vektor(hedef_vektor_dunya, rov.rotation)
        hedef_yerel = Vec3(-hedef_yerel.x, hedef_yerel.y, hedef_yerel.z)

        powers = [v.dot(hedef_yerel) * guc for v in motor_bv]
        if abs(float(hedef_vektor_dunya.y)) <= 1e-6:
            for i in range(4, min(8, len(powers), len(cached_powers))):
                powers[i] = cached_powers[i]
        return powers

    def yaw_gucleri_hesapla(
        self,
        rov=None,
        hedef_vektor_dunya: Vec3 = Vec3(0.0, 0.0, 0.0),
        guc_orani: float = 0.0,
    ):
        if rov is None:
            return [0.0] * 6, 0.0

        motorlar = getattr(rov, "motorlar", [])
        powers = [float(getattr(motor, "guc", 0.0)) for motor in motorlar]
        yaw_indeksleri = range(min(4, len(motorlar)))

        v_rov_dunya = rov.gnc.r_bv
        v_rov_yatay = Vec3(v_rov_dunya.x, 0, v_rov_dunya.z)
        hedef_yatay = Vec3(hedef_vektor_dunya.x, 0, hedef_vektor_dunya.z)
        if v_rov_yatay.length() < 0.001 or hedef_yatay.length() < 0.001:
            return powers, 0.0

        tork_istenen_dunya = v_rov_yatay.cross(hedef_yatay)
        tork_istenen_yerel = self.dunya_to_yerel_vektor(tork_istenen_dunya, rov.rotation)
        for i in yaw_indeksleri:
            powers[i] = motorlar[i].tork_bv.dot(tork_istenen_yerel) * guc_orani
        return powers, 0

    def roll_koru(self, rov=None, guc_orani: float = 1.0):
        if rov is None:
            return [0.0] * 6, False

        guc = self.filo.pid_hesapla(rov,"roll")

        self.filo.roll(rov, guc)

    def pitch_koru(self, rov=None, guc_orani: float = 1.0):
        if rov is None:
            return [0.0] * 6, False

        guc = self.filo.pid_hesapla(rov,"pitch")

        self.filo.pitch(rov, -guc)


        

        

    def yaw(self, rov, guc: float = 0.1):
        motorlar = rov.motorlar
        motorlar[0].calistir(guc)
        motorlar[1].calistir(-guc)
        motorlar[2].calistir(-guc)
        motorlar[3].calistir(guc)

    def roll(self, rov, guc: float = 0.1):
        motorlar = rov.motorlar
        motorlar[4].calistir(guc)
        motorlar[5].calistir(-guc)
        motorlar[6].calistir(guc)
        motorlar[7].calistir(-guc)

    def pitch(self, rov, guc: float = 0.1):
        motorlar = rov.motorlar
        motorlar[4].calistir(guc)
        motorlar[5].calistir(guc)
        motorlar[6].calistir(-guc)
        motorlar[7].calistir(-guc)

    def motorlari_calistir(self, rov_id=0, gucler: list[float] | None = None):
        if gucler is None:
            gucler = []
        motor_listesi = self.filo.motorlar.get(rov_id)
        if not motor_listesi:
            return
        for i in range(len(gucler)):
            motor_listesi[i].calistir(gucler[i])
