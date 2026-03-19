from FiratROVNet.config import HareketAyarlari
import random
import numpy as np
import math
import time


class Formasyon:
    """
    Popüler ve işlevsel 10 formasyon tipini ROV sayısına göre dinamik olarak saklar.
    Her formasyon tipi, lider (index 0) ve takipçiler (index 1+) için pozisyon ofsetlerini döndürür.
    
    Ofsetler (x, y, z) formatında:
    - x: Sağ-sol (pozitif = sağ) - 2D koordinat
    - y: İleri-geri (pozitif = ileri) - 2D koordinat
    - z: Derinlik (pozitif = yukarı, negatif = aşağı) - genelde 0
    """

    def __init__(self,Filo=None):
        self.Filo=Filo
    
    # Formasyon isimleri (20 tip)
    TIPLER = [
        "LINE",          # 0: Çizgi formasyonu (tek sıra)
        "V_SHAPE",       # 1: V şekli (uçan kazlar)
        "DIAMOND",       # 2: Elmas formasyonu
        "SQUARE",        # 3: Kare formasyonu
        "CIRCLE",        # 4: Daire formasyonu
        "ARROW",         # 5: Ok şekli
        "WEDGE",         # 6: Kama şekli
        "ECHELON",       # 7: Eşelon (çapraz sıra)
        "COLUMN",        # 8: Sütun (dikey sıra)
        "SPREAD",        # 9: Yayılım (geniş yayılım)
        "TRIANGLE",      # 10: Üçgen formasyonu
        "CROSS",         # 11: Haç formasyonu
        "STAGGERED",     # 12: Kademeli formasyon
        "WALL",          # 13: Duvar formasyonu
        "STAR",          # 14: Yıldız formasyonu
        "PHALANX",       # 15: Falanks (sıkı düzen, askeri formasyon)
        "RECTANGLE",     # 16: Dikdörtgen formasyonu
        "HEXAGON",       # 17: Altıgen formasyonu
        "WAVE",          # 18: Dalga formasyonu
        "SPIRAL",        # 19: Spiral formasyonu
        "TSHAPE"         # 20: T formasyonu (yeni)
    ]
    
    
   

    def pozisyonlar(self,tip, aralik=15.0, is_3d=False, lider_koordinat=None, yaw=None,g_id=0):
               # 1. GRUP VE ROV LİSTESİNİ AL
        # g_rovs[g_id] bize ROV entity listesini verir: [RovEntity1, RovEntity2...]
        grup_rov_listesi = self.Filo.g_rovs.get(g_id)
        
        if not grup_rov_listesi:
            return {}
            
        n_rovs = len(grup_rov_listesi)
        if n_rovs == 0:
            return {}

        # 2. FORMASYON TİPİNİ BELİRLE
        if isinstance(tip, str):
            tip = tip.upper()
            tip_index = self.TIPLER.index(tip) if tip in self.TIPLER else 0
        else:
            tip_index = int(tip) % len(self.TIPLER)

        # 3. LİDERİ VE REFERANS NOKTASINI BELİRLE
        # Varsayılan lider listenin ilk elemanıdır
        lider_entity = grup_rov_listesi[0]
        lider_id = lider_entity.id
        
        # Grupta 'rol' değeri 1 olan bir ROV var mı diye bak (Entity üzerinden veya Filo verisinden)
        for rov in grup_rov_listesi:
            # Not: Entity içinde .rol attribute'u varsa direkt if rov.rol == 1: kullanılabilir.
            # Biz mevcut yapıya sadık kalarak Filo.get kullanıyoruz:
            if self.Filo.get(rov.id, 'rol') == 1:
                lider_entity = rov
                lider_id = rov.id
                break
        
        # Lider Global Pozisyonu
        if lider_koordinat is not None:
            lider_pos = tuple(map(float, lider_koordinat))
        else:
            # Entity üzerinden gps çekilebiliyorsa: lider_entity.gps
            gps = self.Filo.get(lider_id, "gps")
            lider_pos = (float(gps[0]), float(gps[1]), float(gps[2])) if gps else (0.0, 0.0, 0.0)

        # Lider Yaw Açısı
        if yaw is None:
            yaw = self.Filo.get(lider_id, "yaw") or 0.0

        # 4. YEREL OFSETLERİN HESAPLANMASI
        # Takipçileri ayır (Lider hariç diğer entity'ler)
        takipciler = [rov for rov in grup_rov_listesi if rov.id != lider_id]
        
        # Sonuçları tutacak sözlük: {rov_id: (x, y, z)}
        yerel_ofsetler = {lider_id: (0.0, 0.0, 0.0)}
        
        for idx, rov in enumerate(takipciler):
            # 2D Ofset (x, y)
            lx, ly = self._yerel_xy_hesapla(tip_index, idx, aralik, len(takipciler))
            
            # 3D Ofset (z)
            lz = 0.0
            if is_3d:
                lz = self._yerel_z_hesapla(tip_index, idx, aralik, len(takipciler))
                
            yerel_ofsetler[rov.id] = (lx, ly, lz)

        # 5. GLOBAL KOORDİNATA DÖNÜŞTÜRME (ROTASYON)
        # Yaw açısına göre döndür ve liderin pozisyonuna ekle
        angle_rad = math.radians(yaw)
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        
        global_pozisyonlar = {}
        final_list = []
        
        for rov in grup_rov_listesi:
            lx, ly, lz = yerel_ofsetler[rov.id]
            
            # Rotasyon (X=Sağ, Y=İleri eksenine göre)
            gx = lx * cos_a + ly * sin_a
            gy = -lx * sin_a + ly * cos_a
            
            global_pozisyonlar[rov.id] = (
                lider_pos[0] + gx,
                lider_pos[1] + gy,
                lider_pos[2] + lz
            )

            #Eski yöntem pozisyonları liste şeklinde döndür.
            global_pos = (
                lider_pos[0] + gx,
                lider_pos[1] + gy,
                lider_pos[2] + lz
            )
            final_list.append(global_pos)
            
        return global_pozisyonlar

    def _yerel_xy_hesapla(self, tip, idx, aralik, n_takipci):
        """Formasyon tipine göre X, Y ofsetlerini hesaplar."""
        # Ortak Değişkenler
        row = (idx // 2) + 1
        side = 1 if (idx + 1) % 2 != 0 else -1  # Tekler sağ(1), Çiftler sol(-1)

        if tip == 0:   # LINE
            return (0.0, -aralik * (idx + 1))
            
        elif tip == 1: # V_SHAPE
            row_v = (idx + 2) // 2
            return (side * aralik * row_v, -aralik * row_v)
            
        elif tip == 2: # DIAMOND
            angle = 2 * math.pi * idx / max(n_takipci, 1)
            radius = aralik * (1 + (idx // max(n_takipci, 1)))
            return (radius * math.cos(angle), radius * math.sin(angle))
            
        elif tip == 3: # SQUARE
            side_len = int(math.ceil(math.sqrt(n_takipci)))
            c_row = idx // side_len
            c_col = idx % side_len
            return ((c_col - side_len / 2 + 0.5) * aralik, -c_row * aralik)
            
        elif tip == 4: # CIRCLE
            angle = 2 * math.pi * idx / max(n_takipci, 1)
            radius = aralik * 1.5
            return (radius * math.cos(angle), radius * math.sin(angle))
            
        elif tip == 5: # ARROW
            row_a = idx // 3 + 1
            col_a = (idx % 3) - 1
            return (col_a * aralik * 0.8, -row_a * aralik * 1.2)

        elif tip == 8: # COLUMN
            return (aralik * (idx + 1), 0.0)
            
        elif tip == 10: # TRIANGLE
            satir = int(math.ceil((-1 + math.sqrt(1 + 8 * (idx + 1))) / 2)) - 1
            onceki_toplam = (satir * (satir + 1)) // 2
            pos_in_row = idx - onceki_toplam
            x = (pos_in_row - satir / 2.0) * aralik
            y = -(satir + 1) * aralik
            return (x, y)
            
        elif tip == 20: # TSHAPE
            split_idx = n_takipci // 2
            if idx < split_idx: # Gövde
                return (0.0, -aralik * (idx + 1))
            else: # Baş
                head_idx = idx - split_idx
                h_side = 1 if head_idx % 2 == 0 else -1
                dist = ((head_idx // 2) + 1) * aralik
                return (h_side * dist, 0.0)
                
        # Diğer formasyon tipleri buraya eklenebilir...
        # Fallback (Varsayılan): LINE
        return (0.0, -aralik * (idx + 1))

    def _yerel_z_hesapla(self, tip, idx, aralik, n_takipci):
        """Formasyon tipine göre Z (derinlik) ofsetlerini hesaplar."""
        # Küresel Dağılımlar (CIRCLE, HEXAGON, STAR)
        if tip in [4, 14, 17]:
            vert_angle = math.pi * (idx % 3) / 3 - math.pi / 2
            return aralik * 0.8 * math.sin(vert_angle)
            
        # SPIRAL
        elif tip == 19:
            vert = 2.0 * math.pi * idx / max(n_takipci, 1)
            return -aralik * 0.4 * math.sin(vert)
            
        # WAVE
        elif tip == 18:
            vert = 2.0 * math.pi * idx / max(n_takipci, 1)
            return -aralik * 0.3 * math.cos(vert)
            
        # TRIANGLE (Piramit yapısı)
        elif tip == 10:
            satir = int(math.ceil((-1 + math.sqrt(1 + 8 * (idx + 1))) / 2)) - 1
            return -(satir * aralik * 0.4)
            
        # Varsayılan (Kademeli derinlik)
        katman = idx // 3
        return -katman * aralik * 0.5


class FormationMixin:
    """Formasyon mantığı."""

    def formasyon(self, formasyon_id="LINE", aralik=None, is_3d=False, lider_koordinat=None, dinamik=True):
        if aralik is None:
            aralik = HareketAyarlari.FORMASYON_VARSAYILAN_ARALIK
        formasyon_obj = Formasyon(self.filo)
        pozisyonlar = formasyon_obj.pozisyonlar(formasyon_id, aralik, is_3d=is_3d, lider_koordinat=lider_koordinat)

        if not pozisyonlar or len(pozisyonlar) == 0:
            print("❌ [FORMASYON] Pozisyonlar alınamadı!")
            return None if lider_koordinat is not None else None

        aktif_rovs = self.filo.rovs if hasattr(self.filo, 'rovs') else [r for r in self.filo.ortam_ref.rovs if r]
        if len(pozisyonlar) != len(aktif_rovs):
            print(f"⚠️ [FORMASYON] Uyarı: Pozisyon sayısı ({len(pozisyonlar)}) ROV sayısı ({len(aktif_rovs)}) ile eşleşmiyor!")

        if lider_koordinat is not None:
            ursina_positions = []
            for pozisyon in pozisyonlar:
                config_x, config_y, config_z = pozisyon
                ursina_positions.append((config_x, config_y, config_z))
            print(f"✅ [FORMASYON] Pozisyonlar hesaplandı: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
            return [(x, z, y) for x, y, z in ursina_positions]

        if dinamik:
            self.filo.aktif_formasyon = {
                'id': formasyon_id,
                'aralik': aralik,
                'is_3d': is_3d
            }
        else:
            self.filo.aktif_formasyon = None

        lider_id = None
        for rov in aktif_rovs:
            if hasattr(rov, 'role') and rov.role == 1:
                lider_id = rov.id
                break

        for idx, pozisyon in enumerate(pozisyonlar):
            if idx >= len(aktif_rovs):
                break
            rov = aktif_rovs[idx]
            if lider_id is not None and rov.id == lider_id:
                mevcut_hedef = self.filo.hedef(rov_id=rov.id)
                if mevcut_hedef is not None:
                    print(f"ℹ️ [FORMASYON] ROV-{rov.id} (Lider) hareket halinde, mevcut hedefine devam ediyor.")
                    continue

            sim_x, sim_y, sim_z = pozisyon
            if sim_z >= 0:
                sim_z = -10.0
            try:
                self.filo.git(rov.id, sim_x, sim_y, sim_z, ai=True)
                print(f"✅ [FORMASYON] ROV-{rov.id} hedefi ayarlandı: ({sim_x:.2f}, {sim_y:.2f}, {sim_z:.2f})")
            except Exception as e:
                print(f"⚠️ [FORMASYON] ROV-{rov.id} için hedef ayarlanırken hata: {e}")

        print(f"✅ [FORMASYON] Formasyon kuruldu: Tip={formasyon_id}, Aralık={aralik}, ROV Sayısı={len(pozisyonlar)}")
        return None

    def _formasyon_sec_impl(self, margin=None, is_3d=False, offset=None, sessiz=True, dinamik=True, tekrar=1, g_id=0):
        self.formasyon_sec_tekrar += 1
        if self.formasyon_sec_tekrar<tekrar:
            return None
        self.formasyon_sec_tekrar=0
        initial_margin = margin if margin is not None else HareketAyarlari.FORMASYON_OFFSET-10
        min_aralik = HareketAyarlari.FORMASYON_MIN_ARALIK
        offset = offset if offset is not None else HareketAyarlari.FORMASYON_OFFSET-10
        lider_id = None
        lider_gps = None
        lider_hareket_halinde = False
        try:
            self.filo._formasyon_hedefleri.clear()
            lider_bilgi = self.find_leader_info(sessiz=sessiz, g_id=g_id)
            lider_id = lider_bilgi[0] if lider_bilgi else None
            lider_gps = lider_bilgi[1] if lider_bilgi else None
            if lider_id is None:
                if not sessiz:
                    print(f"❌ [FORMASYON] Grup-{g_id} icin lider bulunamadi.")
                self.cache_formasyon_result(None)
                return None

            lider_mevcut_hedef = self.filo.hedef(rov_id=lider_id)
            lider_hareket_halinde = lider_mevcut_hedef is not None

            yasakli_noktalar = self.filo.ada_cevre()
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'engel_bulutu'):
                engel_bulutu = getattr(self.filo.ortam_ref, 'engel_bulutu', None) or []
                dinamik_engeller = [[float(p[0]), float(p[1])] for p in engel_bulutu if p is not None and len(p) >= 2]
                yasakli_noktalar.extend(dinamik_engeller)

            hull_data = self.yeni_hull(yasakli_noktalar=yasakli_noktalar, offset=offset,g_id=g_id) or {}
            hull_obj = hull_data.get("hull") if isinstance(hull_data, dict) else None
            hull_merkez = hull_data.get("center") if isinstance(hull_data, dict) else None
            if not hull_obj or not hull_merkez:
                if not sessiz:
                    print(f"⚠️ [FORMASYON] Grup-{g_id} icin hull olusturulamadi, lider merkezli fallback uygulanacak.")
                return self._uygula_fallback_formasyon(
                    lider_id=lider_id,
                    lider_gps=lider_gps,
                    lider_hareket_halinde=lider_hareket_halinde,
                    is_3d=is_3d,
                    dinamik=dinamik,
                    sessiz=sessiz,
                    g_id=g_id,
                    sebep="Hull olusturulamadi",
                )

            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'minimap'):
                m_ui = self.filo.ortam_ref.minimap
                if m_ui:
                    if hasattr(m_ui, 'goster'): m_ui.goster(True)
                    if hasattr(m_ui, 'update_hull'): m_ui.update_hull(hull_obj)

            ref_pos_3d = lider_gps if lider_gps else (hull_merkez[0], hull_merkez[1], 0.0)
            start_pos_2d, unit_dir_2d, total_dist = self.generate_search_points(ref_pos_3d, hull_merkez)

            denenecek_ids = self.get_formation_ids_to_try()
            yaw_secenekleri = [0, 90, 180, 270]
            formasyon_motoru = Formasyon(self.filo)
            best_overall = None
            low_d, high_d = 0.0, total_dist
            for _ in range(7): 
                mid_d = (low_d + high_d) / 2
                curr_2d = start_pos_2d + unit_dir_2d * mid_d
                merkez_3d = (float(curr_2d[0]), float(curr_2d[1]), float(ref_pos_3d[2]))
                found_at_this_pos = False
                for deneme_yaw in yaw_secenekleri:
                    for f_id in denenecek_ids:
                        low_a, high_a = min_aralik, initial_margin
                        current_best_a = -1
                        current_best_p = None
                        while low_a <= high_a:
                            mid_a = (low_a + high_a) / 2
                            p = formasyon_motoru.pozisyonlar(f_id, mid_a, is_3d, merkez_3d, deneme_yaw,g_id)
                            if p and self.filo.hull_manager.formasyon_gecerli_mi(p, hull_obj, mid_a,g_id):
                                current_best_a = mid_a
                                current_best_p = p
                                low_a = mid_a + 1.0
                            else:
                                high_a = mid_a - 1.0
                        if current_best_a != -1:
                            best_overall = {
                                'f_id': f_id, 'aralik': current_best_a, 'yaw': deneme_yaw,
                                'merkez': merkez_3d, 'pozisyonlar': current_best_p
                            }
                            found_at_this_pos = True
                            break
                    if found_at_this_pos: break
                if found_at_this_pos:
                    high_d = mid_d - 2.0
                else:
                    low_d = mid_d + 2.0

            if best_overall:
                b = best_overall
                self._apply_formation_results(
                    b['f_id'], b['aralik'], b['yaw'], b['merkez'], 
                    b['pozisyonlar'], lider_id, is_3d, dinamik, sessiz, lider_hareket_halinde,g_id
                )
                if not sessiz:
                    durum = "Dinamik" if dinamik else "Sabit"
                    print(f"✅ [MİNİMAP] {durum} {b['f_id']} seçildi. Alan Cyan olarak işlendi.")
                
                # 🔹 FORMASYON + HULL BİLGİLERİ SÖZLÜĞÜ
                hull_information_dict = {
                    'hull': hull_obj,                           # Hull Shapely objesi
                    'center': hull_merkez,                      # Hull merkezi (x, y)
                    'hull_data': hull_data,                     # Tüm hull data
                    'offset': offset,                           # Hull offset
                    'yasakli_noktalar': yasakli_noktalar       # Engeller listesi
                }
                
                formasyon_information_dict = {
                    'formasyon_id': str(b['f_id']),            # Formasyon tipi adı
                    'formasyon_index': int(b['f_id']),         # Formasyon indeksi
                    'aralik': round(float(b['aralik']), 1),    # Araç arası mesafe
                    'merkez': (round(b['merkez'][0], 2), round(b['merkez'][1], 2)),  # Formasyon merkezi
                    'yaw': float(b['yaw']),                    # Yaw açısı
                    'pozisyonlar': b['pozisyonlar'],          # ROV pozisyonları
                    'lider_id': lider_id,                      # Lider ROV ID
                    'grup_id': g_id                            # Grup ID
                }
                
                # 🔹 SONUCU CACHE'E KAY
                result = {
                    'f_id': int(b['f_id']),
                    'aralik': round(float(b['aralik']), 1),
                    'merkez': (round(b['merkez'][0], 2), round(b['merkez'][1], 2)),
                    'yaw': float(b['yaw']),
                    'hull_information': hull_information_dict,           # Hull verisi
                    'formasyon_information': formasyon_information_dict  # Formasyon verisi
                }
                self.cache_formasyon_result(result)
                
                # 🔹 HULL INFORMATION'I OTOMATIK TETIKLE (JSON'a kaydet)
                # Hull merkez kontrol: 25m'den az değişirse kaydetme
                try:
                    if self.filo and hasattr(self.filo, 'get_hull_information'):
                        self.filo.get_hull_information(sample_count=50, g_id=g_id, kayit=True, sessiz=True, offset_threshold=25.0)
                except Exception as e:
                    self.filo.ds = e  # Hata bilgisini filo'ya kaydet (debug için)
                
                return result
            

            
            # Hull-uyumlu en iyi formasyon bulunamazsa, lider etrafinda
            # temel bir formasyon fallback'i uygula ki komut bosa gitmesin.
            fallback_result = self._uygula_fallback_formasyon(
                lider_id=lider_id,
                lider_gps=lider_gps,
                lider_hareket_halinde=lider_hareket_halinde,
                is_3d=is_3d,
                dinamik=dinamik,
                sessiz=sessiz,
                g_id=g_id,
                sebep="Optimum secim bulunamadi",
            )
            if fallback_result is not None:
                return fallback_result

            # 🔹 BAŞARISIZ SONUCU DA CACHE'E KAY
            self.cache_formasyon_result(None)
            if not sessiz:
                print(f"❌ [FORMASYON] Grup-{g_id} icin uygulanabilir formasyon bulunamadi.")
            return None
        except Exception as e:
            self.filo.ds=e
            if not sessiz:
                print(f"❌ [FORMASYON] Grup-{g_id} secimi sirasinda hata: {e}")
            fallback_result = self._uygula_fallback_formasyon(
                lider_id=lider_id,
                lider_gps=lider_gps,
                lider_hareket_halinde=lider_hareket_halinde,
                is_3d=is_3d,
                dinamik=dinamik,
                sessiz=sessiz,
                g_id=g_id,
                sebep="Hata nedeniyle fallback",
            )
            if fallback_result is not None:
                return fallback_result
            self.cache_formasyon_result(None)  # 🔹 HATA DURUMUNDA DA CACHE'E YAZ
            return None

    def _sayisal_degerler_gecerli_mi(self, *values) -> bool:
        for value in values:
            try:
                if not math.isfinite(float(value)):
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _uygula_fallback_formasyon(
        self,
        lider_id,
        lider_gps,
        lider_hareket_halinde,
        is_3d,
        dinamik,
        sessiz,
        g_id=0,
        sebep="Fallback",
    ):
        if lider_id is None:
            self.cache_formasyon_result(None)
            return None

        fallback_yaw = self.filo.get(lider_id, "yaw") or 0.0
        fallback_merkez = lider_gps if lider_gps else (0.0, 0.0, -10.0)
        fallback_aralik = max(
            float(HareketAyarlari.FORMASYON_MIN_ARALIK),
            float(HareketAyarlari.FORMASYON_VARSAYILAN_ARALIK),
        )
        fallback_f_id = 0  # LINE
        fallback_pozisyonlar = Formasyon(self.filo).pozisyonlar(
            fallback_f_id,
            fallback_aralik,
            is_3d,
            fallback_merkez,
            fallback_yaw,
            g_id,
        )

        if not fallback_pozisyonlar:
            self.cache_formasyon_result(None)
            if not sessiz:
                print(f"❌ [FORMASYON] Grup-{g_id} icin fallback da uygulanamadi.")
            return None

        self._apply_formation_results(
            fallback_f_id,
            fallback_aralik,
            fallback_yaw,
            fallback_merkez,
            fallback_pozisyonlar,
            lider_id,
            is_3d,
            dinamik,
            sessiz,
            lider_hareket_halinde,
            g_id,
        )
        result = {
            'f_id': int(fallback_f_id),
            'aralik': round(float(fallback_aralik), 1),
            'merkez': (round(float(fallback_merkez[0]), 2), round(float(fallback_merkez[1]), 2)),
            'yaw': float(fallback_yaw),
            'fallback': True,
            'sebep': sebep,
        }
        self.cache_formasyon_result(result)
        if not sessiz:
            print(f"⚠️ [FORMASYON] {sebep}, fallback LINE uygulandi (Grup-{g_id}).")
        return result

    def _apply_formation_results(self, f_id, aralik, yaw, merkez, pozisyonlar, lider_id, is_3d, dinamik, sessiz, lider_hareket_halinde, g_id=0):
        group_rov_list = self.filo.g_rovs.get(g_id)
        if not group_rov_list:
            if not sessiz: print(f"⚠️ [FORMASYON] Grup-{g_id} bulunamadı veya boş.")
            return
        if not self._sayisal_degerler_gecerli_mi(aralik, yaw):
            if not sessiz:
                print(f"❌ [FORMASYON] Geçersiz formasyon parametresi: aralık={aralik}, yaw={yaw}")
            return
        target_positions = {}
        if isinstance(pozisyonlar, dict):
            target_positions = pozisyonlar
        elif isinstance(pozisyonlar, list):
            for idx, rov in enumerate(group_rov_list):
                if idx < len(pozisyonlar):
                    target_positions[rov.id] = pozisyonlar[idx]
        if dinamik:
            self.filo.aktif_formasyon[g_id] = {
                'id': f_id, 
                'aralik': aralik, 
                'is_3d': is_3d,
                'yaw': yaw,
                'g_id': g_id
            } 
        else:
            self.filo.aktif_formasyon[g_id] = None
        for r_id, pos in target_positions.items():
            if r_id == lider_id and lider_hareket_halinde:
                if not sessiz: 
                    print(f"ℹ️ [FORMASYON] Lider ROV-{r_id} (Grup-{g_id}) görevine devam ediyor.")
                continue
            try:
                sim_x, sim_y, sim_z = pos
            except (TypeError, ValueError):
                print(f"❌ [HATA] ROV-{r_id} için geçersiz pozisyon verisi: {pos}")
                continue
            if not self._sayisal_degerler_gecerli_mi(sim_x, sim_y, sim_z):
                if not sessiz:
                    print(f"❌ [FORMASYON] ROV-{r_id} için NaN/inf pozisyon atlandı: {pos}")
                continue
            final_z = -10.0 if sim_z >= 0 else sim_z
            if r_id != lider_id:
                if hasattr(self.filo, '_formasyon_hedefleri'):
                    self.filo._formasyon_hedefleri[r_id] = {
                        'pozisyon': (sim_x, sim_y, final_z), 
                        'hedef_yaw': yaw
                    }

            # Mod==0 kontrolu: ROV'u bul ve mod durumunu kontrol et
            rov = self.filo.find_rov_by_id(r_id)
            if rov and rov.gnc.mod == 0:
                continue

            self.filo.git(r_id, sim_x, sim_y, final_z, ai=True, sessiz=sessiz)

    def find_leader_info(self, sessiz: bool = False, g_id: int = 0) -> tuple:
        def _gps_al(rov):
            gps = None
            if hasattr(self.filo, 'get'):
                gps = self.filo.get(rov.id, "gps")
            elif hasattr(rov, 'gps'):
                gps = rov.gps
            if gps is None:
                return None
            try:
                return (float(gps[0]), float(gps[1]), float(gps[2]))
            except (IndexError, TypeError, ValueError):
                if not sessiz:
                    print(f"⚠️ [UYARI] Lider ROV-{rov.id} GPS verisi bozuk: {gps}")
                return (0.0, 0.0, 0.0)

        rov_listesi = self.filo.g_rovs.get(g_id)
        if not rov_listesi:
            if not sessiz:
                print(f"⚠️ [FORMASYON] Grup-{g_id} listesi boş veya bulunamadı!")
            return None, None

        # Lider bilgisini once leader_manager cache'inden çöz.
        # Rol alanı senkron dışı kaldığında formasyon tarafı buradan kopuyordu.
        leader_manager = getattr(self.filo, 'leader_manager', None)
        if leader_manager is not None:
            mevcut_liderler = getattr(leader_manager, 'mevcut_lider_id', {})
            aday_lider_id = mevcut_liderler.get(g_id)
            if isinstance(aday_lider_id, int) and aday_lider_id >= 0:
                for rov in rov_listesi:
                    if rov is None or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                        continue
                    if getattr(rov, 'id', None) == aday_lider_id:
                        lider_gps = _gps_al(rov)
                        if lider_gps is None:
                            if not sessiz:
                                print(f"⚠️ [UYARI] Lider ROV-{aday_lider_id} GPS verisi yok!")
                            return aday_lider_id, None
                        return aday_lider_id, lider_gps

        lider_rov_id = None
        lider_gps = None
        for rov in rov_listesi:
            if rov is None:
                continue
            rol = -1
            if hasattr(rov, 'role'):
                rol = rov.role
            elif hasattr(rov, 'rol'):
                rol = rov.rol
            elif hasattr(rov, 'get'):
                rol = rov.get("rol")
            elif hasattr(rov, 'gnc') and hasattr(rov.gnc, 'rol'):
                rol = rov.gnc.rol
            if rol == 1:
                lider_rov_id = rov.id
                lider_gps = _gps_al(rov)
                if lider_gps is None and not sessiz:
                    print(f"⚠️ [UYARI] Lider ROV-{rov.id} GPS verisi yok!")
                if leader_manager is not None and hasattr(leader_manager, 'mevcut_lider_id'):
                    leader_manager.mevcut_lider_id[g_id] = lider_rov_id
                break

        if lider_rov_id is None and not sessiz:
            if hasattr(self.filo, 'ds') and self.filo.ds:
                print(f"Debug Info: {self.filo.ds}")
            print(f"❌ [HATA] Grup-{g_id} içinde Lider ROV tespit edilemedi.")
        return lider_rov_id, lider_gps

    def get_formation_ids_to_try(self) -> list:
        denenecek_formasyon_idleri = []
        pool_kopyasi = self.filo._formasyon_id_pool.copy()
        while len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER) and len(pool_kopyasi) > 0:
            denenecek_formasyon_idleri.append(pool_kopyasi.pop(0))
        if len(denenecek_formasyon_idleri) < len(Formasyon.TIPLER):
            kalan_idler = [i for i in range(len(Formasyon.TIPLER)) if i not in denenecek_formasyon_idleri]
            random.shuffle(kalan_idler)
            denenecek_formasyon_idleri.extend(kalan_idler)
        return denenecek_formasyon_idleri
    
    def cache_formasyon_result(self, result, sessiz=True):
        """
        🔹 Formasyon seçim sonuçlarını cache'e kaydeder (async/worker friendly)
        
        Args:
            result: Dict with {f_id, aralik, merkez, yaw} or None if failed
            sessiz: True ise log yazma
        """
        self.last_formasyon_result = result
        self.formasyon_result_timestamp = time.time()
    
    def get_formasyon_result(self, clear=False):
        """
        🔹 Cache'deki son formasyon seçim sonucunu al
        
        Args:
            clear: True ise sonucu al ve temizle
        
        Returns:
            Dict or None; timestamp ile birlikte
        """
        result = {
            'sonuc': self.last_formasyon_result,
            'zaman': self.formasyon_result_timestamp
        }
        if clear:
            self.last_formasyon_result = None
            self.formasyon_result_timestamp = None
        return result
    def generate_search_points(self, lider_gps, hull_merkez):
        lider_pos_2d = np.array([float(lider_gps[0]), float(lider_gps[1])])
        merkez_pos_2d = np.array([float(hull_merkez[0]), float(hull_merkez[1])])
        vektor = merkez_pos_2d - lider_pos_2d
        toplam_mesafe = np.linalg.norm(vektor)
        if toplam_mesafe < 0.1:
            return lider_pos_2d, np.array([0.0, 0.0]), 0.0
        birim_yon = vektor / toplam_mesafe
        return lider_pos_2d, birim_yon, toplam_mesafe
