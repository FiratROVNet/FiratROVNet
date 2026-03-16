import math
from ursina import Entity, color, destroy, Text
from FiratROVNet.config import HareketAyarlari, HavuzAyarlari


class VisualizationMixin:
    """Gorsellestirme araclari."""

    def minimap(self, durum=True, convex=True, a_star=True, scale=None, grid=None, *args, **kwargs):
        """
        Minimap'i acar, kapatir veya durumunu dondurur.
        scale: carpan (1=taban 0.45, 2=2 kati, 0.1=4.5 vb.); verilirse boyut dinamik guncellenir.
        grid: grid sayisi (None=varsayilan GRID_UNIT m; N=toplam N aralik, 1 grid=(2*havuz)/N m).
        filo.minimap("ekle", filo.ada_cevre()) ile ada cevre noktalarini minimapte turuncu-kahverengi cizgi olarak gosterir.
        """
        if durum == "ekle" and convex is not None and hasattr(convex, '__iter__') and not isinstance(convex, (bool, str)):
            points = list(convex)
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
                if hasattr(self.filo.ortam_ref.minimap, 'update_ada_cevre'):
                    self.filo.ortam_ref.minimap.update_ada_cevre(points)
                    print(f"✅ [MINIMAP] Ada cevre noktalari guncellendi: {points}")
                    if not self.filo.ortam_ref.minimap.visible:
                        self.filo.ortam_ref.minimap.goster(True)
                else:
                    print("⚠️ [MINIMAP] update_ada_cevre bulunamadi.")
            else:
                print("❌ [MINIMAP] Minimap sistemi bulunamadi!")
            return

        if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            if not hasattr(self.filo.ortam_ref.minimap, 'filo_ref') or self.filo.ortam_ref.minimap.filo_ref != self.filo:
                self.filo.ortam_ref.minimap.filo_ref = self.filo

            if durum is None:
                if scale is not None or grid is not None:
                    self.filo.ortam_ref.minimap.goster(
                        self.filo.ortam_ref.minimap.visible, convex, a_star, scale=scale, grid=grid
                    )
                self.filo.ortam_ref.minimap.visible = not self.filo.ortam_ref.minimap.visible
                status = "ACIK" if self.filo.ortam_ref.minimap.visible else "KAPALI"
                print(f"🗺️ [MINIMAP] Minimap su an {status}")
            else:
                self.filo.ortam_ref.minimap.goster(durum, convex, a_star, scale=scale, grid=grid)
        else:
            print("❌ [MINIMAP] Minimap sistemi bulunamadi!")

    def hedef_gorsel_olustur(self, x, y, z, id=None, debug=True):
        """
        Hedef pozisyonunu hem 3D dunyada hem de Minimap uzerinde gosterir.
        x, y, z: Simulasyon koordinatlari (Koordinator.sim_to_ursina tarafindan cevrilmis).
        id: Noktanin kimligi (debug=False ise zorunludur).
        debug=True: Gecici kirmizi X isareti (Her yeni hedefte eskiyi siler).
        debug=False: Kalici, ID numarali mavi cember (Ekranda kalir).
        """
        # Koordinat hazirligi
        x_urs, y_urs, z_urs = x, z, y

        if not self.filo.ortam_ref:
            return

        if not hasattr(self.filo, 'kalici_hedefler'):
            self.filo.kalici_hedefler = {}

        # Gecici hedef: create-once, her cagrida sadece konum guncelle
        if debug:
            x_boyutu = getattr(HareketAyarlari, 'HEDEF_X_BOYUTU', 15)
            kalinlik = getattr(HareketAyarlari, 'HEDEF_KALINLIK', 0.5)
            if self.filo.hedef_gorsel is None:
                self.filo.hedef_gorsel = Entity()
                Entity(model='cube', rotation=(90, 0, 45), scale=(x_boyutu, kalinlik, kalinlik),
                       color=color.rgba(255, 0, 0, 0.5), parent=self.filo.hedef_gorsel, unlit=True)
                Entity(model='cube', rotation=(90, 0, -45), scale=(x_boyutu, kalinlik, kalinlik),
                       color=color.rgba(255, 0, 0, 0.5), parent=self.filo.hedef_gorsel, unlit=True)
                Entity(model='sphere', scale=(2, 2, 2), color=color.rgba(255, 0, 0, 0.5),
                       parent=self.filo.hedef_gorsel, unlit=True)
                Entity(model='circle', rotation=(90, 0, 0), scale=(x_boyutu * 1.5, x_boyutu * 1.5, 1),
                       color=color.rgb(0, 255, 120), parent=self.filo.hedef_gorsel, unlit=True, wireframe=True)
            self.filo.hedef_gorsel.position = (x_urs, y_urs, z_urs)
            self.filo.hedef_gorsel.enabled = True

        # Kalici hedef
        else:
            if id is None:
                print("⚠️ Hata: debug=False iken bir id belirtmelisiniz!")
                return

            self.hedef_sil(id)

            yeni_hedef = Entity(position=(x_urs, y_urs, z_urs))

            Entity(model='circle', parent=yeni_hedef, rotation=(90, 0, 0), scale=5,
                   color=color.cyan, wireframe=True, unlit=True)

            Text(text=str(id), parent=yeni_hedef, y=1.5, scale=25,
                 color=color.yellow, origin=(0, 0), billboard=True)

            self.kalici_hedefler[id] = yeni_hedef

        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedef_isaretle(x_urs, z_urs, id=id, debug=debug)

    def hedef_sil(self, id):
        """Spesifik bir ID'ye sahip hedefi hem 3D dunyadan hem de Minimap'ten temizler."""
        if hasattr(self, 'kalici_hedefler') and id in self.kalici_hedefler:
            destroy(self.kalici_hedefler[id])
            del self.kalici_hedefler[id]

        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedef_sil(id)

    def debug_hedefleri_temizle(self):
        """Ekrandaki tum kalici (ID'li) hedefleri ve gecici hedefleri temizler."""
        if hasattr(self, 'kalici_hedefler'):
            ids = list(self.kalici_hedefler.keys())
            for hid in ids:
                self.hedef_sil(hid)

        if self.filo.hedef_gorsel:
            destroy(self.filo.hedef_gorsel)
            self.filo.hedef_gorsel = None

        if hasattr(self.filo.ortam_ref, 'minimap') and self.filo.ortam_ref.minimap:
            self.filo.ortam_ref.minimap.hedefleri_temizle()

    def get_apf_vektor_verts_list(self, minimap):
        """
        APF vektor listesi icin minimap kose ve renk listesi dondurur.
        """
        apf_list = getattr(self, '_apf_vektor_list', None)
        if not apf_list:
            return []
        z_line = -0.37
        havuz_genisligi = getattr(minimap, 'havuz_genisligi', HavuzAyarlari.HAVUZ_GENISLIK)
        sonuc = []
        for item in apf_list:
            p1_xyz = item.get('baslangic')
            p2_xyz = item.get('bitis')
            renk = item.get('renk', 'm')
            uzunluk_metre = item.get('uzunluk', 20.0)
            reverse = item.get('reverse', False)
            if p1_xyz is None or p2_xyz is None or len(p1_xyz) < 2 or len(p2_xyz) < 2:
                continue
            p1 = minimap.dunya_to_harita(float(p1_xyz[0]), float(p1_xyz[1]))
            p2 = minimap.dunya_to_harita(float(p2_xyz[0]), float(p2_xyz[1]))
            verts = self._vektor_verts_birim(p1, p2, z_line, havuz_genisligi, uzunluk_metre=uzunluk_metre, reverse=reverse)
            sonuc.append((verts, renk))
        return sonuc

    def get_vektor_renk(self):
        """Minimap vektor cizgisi icin renk kodu doner: k, y, m, s, t (varsayilan m)."""
        return getattr(self, '_vektor_renk', 'm')

    def _vektor_verts_birim(self, p1, p2, z_line=-0.37, havuz_genisligi=None, uzunluk_metre=None, reverse=False):
        """
        Harita koordinatinda p1 -> p2 yonunde (reverse=True ise ters yonde) sabit uzunlukta vektor kose listesi doner.
        """
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        d = math.sqrt(dx * dx + dy * dy)
        if d >= 1e-9:
            ux, uy = dx / d, dy / d
            if reverse:
                ux, uy = -ux, -uy
            if havuz_genisligi is None:
                havuz_genisligi = HavuzAyarlari.HAVUZ_GENISLIK
            h = max(float(havuz_genisligi), 1.0)
            if uzunluk_metre is None:
                uzunluk_metre = getattr(self, '_vektor_uzunluk_metre', 10.0)
            birim_uzunluk = float(uzunluk_metre) / (2.0 * h)
            ex = p1.x + ux * birim_uzunluk
            ey = p1.y + uy * birim_uzunluk
            vektor_aci = math.atan2(uy, ux)
            kanat_uzunluk = birim_uzunluk / 6.0
            aci_145 = vektor_aci + math.radians(145)
            aci_225 = vektor_aci + math.radians(225)
            w1x = ex + kanat_uzunluk * math.cos(aci_145)
            w1y = ey + kanat_uzunluk * math.sin(aci_145)
            w2x = ex + kanat_uzunluk * math.cos(aci_225)
            w2y = ey + kanat_uzunluk * math.sin(aci_225)
            # Titremeyi onlemek icin koordinatlari yuvarla (minimap APF sabit gorunur)
            r3 = lambda x: round(float(x), 3)
            return [
                (r3(p1.x), r3(p1.y), z_line), (r3(ex), r3(ey), z_line),
                (r3(ex), r3(ey), z_line), (r3(w1x), r3(w1y), z_line),
                (r3(ex), r3(ey), z_line), (r3(w2x), r3(w2y), z_line),
            ]
        r3 = lambda x: round(float(x), 3)
        return [(r3(p1.x), r3(p1.y), z_line), (r3(p1.x), r3(p1.y), z_line)]
