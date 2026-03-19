"""
🎯 Hull Information Manager
Kapsamlı hull bilgisi: hull merkezi, sampled points, formasyon & grup detayları
Modüler yapı: GNC subpackage içinde bağımsız yönetici
"""

import json
from datetime import datetime


class HullInformationManager:
    """Hull + Formasyon + Grup bilgisini yönetir."""
    
    def __init__(self, filo_ref):
        """
        Args:
            filo_ref: Filo sınıfı referansı
        """
        self.filo = filo_ref
        
        # Cache attributes
        self.last_hull_information = None
        self.hull_information_timestamp = None
        self.last_hull_center = None  # 🔹 Önceki hull merkez (offset karşılaştırma için)
        self._ortam_id = None
    
    def grup_bilgisi_al(self, group_id):
        """
        🔹 Belirli bir gruba ait tüm ROV'ların detaylı bilgilerini döner
        
        Args:
            group_id: Grup ID'si
        
        Returns:
            Dict: Grup ROV'ları, lider, pil durumu, vb. detaylar
        """
        if not self.filo or not self.filo.g_rovs:
            return None
        
        grupla_rovlar = self.filo.g_rovs.get(group_id, [])
        if not grupla_rovlar:
            return None
        
        grup_info = {
            'group_id': group_id,
            'rov_sayisi': len(grupla_rovlar),
            'rovlar': [],
            'lider_id': None,
            'lider_yaw': None,
            'toplam_batarya': 0.0,
            'ortalama_batarya': 0.0,
            'rov_idleri': []
        }
        
        lider_bulundu = False
        
        for rov in grupla_rovlar:
            if not rov or (hasattr(rov, 'is_destroyed') and rov.is_destroyed):
                continue
            
            rov_info = {
                'rov_id': rov.id,
                'group_id': rov.group_id,
                'pozisyon': {
                    'x': float(rov.x),
                    'y': float(rov.y),
                    'z': float(rov.z)
                },
                'batarya': float(rov.battery),
                'rol': int(rov.role),  # 0=normal, 1=lider
                'yaw': float(rov.rotation_y),
                'sonar': float(rov.son_sonar_mesafesi),
                # 🔹 Merkezi lidar kaynakları: rov.l0, rov.l1, rov.l2, rov.l3 properties kullan
                'lidar': {
                    0: float(rov.l0),
                    1: float(rov.l1),
                    2: float(rov.l2),
                    3: float(rov.l3),
                } if (hasattr(rov, 'l0') and hasattr(rov, 'l1') and hasattr(rov, 'l2') and hasattr(rov, 'l3')) else dict(rov.son_lidar_mesafeleri) if isinstance(rov.son_lidar_mesafeleri, dict) else {}
            }
            
            # GNC sistemi varsa ek bilgiler
            if hasattr(rov, 'gnc') and rov.gnc:
                rov_info['gnc_mode'] = getattr(rov.gnc, 'mod', 0)
                rov_info['gps_sinyal'] = getattr(rov.gnc, 'gps_sinyal', 0)
            else:
                rov_info['gnc_mode'] = 0
                rov_info['gps_sinyal'] = 0
            
            # Lider tespiti
            if rov.role == 1:  # Lider ise
                grup_info['lider_id'] = rov.id
                grup_info['lider_yaw'] = float(rov.rotation_y)
                lider_bulundu = True
            
            grup_info['rovlar'].append(rov_info)
            grup_info['rov_idleri'].append(rov.id)
            grup_info['toplam_batarya'] += float(rov.battery)
        
        if grup_info['rov_sayisi'] > 0:
            grup_info['ortalama_batarya'] = round(grup_info['toplam_batarya'] / len(grup_info['rovlar']), 2)
        
        return grup_info
    
    def formasyon_rov_pozisyonlari_al(self, formasyon_id, formasyon_merkez, formasyon_yaw, grup_rovlar):
        """
        🔹 Formasyon şablonundan ROV'ların formasyondaki (hedef) pozisyonlarını hesapla
        
        Args:
            formasyon_id: Formasyon türü ('LINE', 'CIRCLE', vb.)
            formasyon_merkez: Formasyon merkezi [x, y]
            formasyon_yaw: Formasyon yaw açısı (derece)
            grup_rovlar: Gruptaki ROV ID'leri listesi
        
        Returns:
            Dict: ROV ID -> hedef pozisyon ([x, y]) mapping'i
                  Örnek: {0: [100.5, 200.2], 1: [105.3, 198.1], ...}
        """
        try:
            formasyon_pozisyonlar = {}
            
            if not formasyon_merkez or formasyon_id == 'UNKNOWN':
                # Merkez pozisyon hepsine
                for rov_id in grup_rovlar:
                    formasyon_pozisyonlar[rov_id] = [float(formasyon_merkez[0]), float(formasyon_merkez[1])]
                return formasyon_pozisyonlar
            
            # Formasyon şablonundan pozisyonları çıkar
            formasyon_result = self.filo.helper.get_formasyon_result(clear=False)
            if not formasyon_result:
                for rov_id in grup_rovlar:
                    formasyon_pozisyonlar[rov_id] = [float(formasyon_merkez[0]), float(formasyon_merkez[1])]
                return formasyon_pozisyonlar
            
            formasyon_data = formasyon_result.get('sonuc', {})
            formasyon_info = formasyon_data.get('formasyon_information', {})
            
            # 🔹 Template'deki pozisyonlar (STRING KEY'lerle: '0', '1', '2'...)
            # Bu pozisyonlar ABSOLUTE koordinatlardır (merkeze relative değil)
            pozisyon_template = formasyon_info.get('pozisyonlar', {})  # {'0': [x,y,z], '1': [x,y,z], ...}
            
            # Her ROV için hedef pozisyonu al (template'den doğrudan)
            for idx, rov_id in enumerate(grup_rovlar):
                # String key ile template'den al
                template_key = str(idx)
                
                if template_key in pozisyon_template:
                    # Template'deki 3D pozisyondan 2D al
                    template_pos = pozisyon_template[template_key]
                    if isinstance(template_pos, (list, tuple)) and len(template_pos) >= 2:
                        formasyon_pozisyonlar[rov_id] = [float(template_pos[0]), float(template_pos[1])]
                    else:
                        # Fallback: merkez pozisyon
                        formasyon_pozisyonlar[rov_id] = [float(formasyon_merkez[0]), float(formasyon_merkez[1])]
                else:
                    # Fallback: merkez pozisyon
                    formasyon_pozisyonlar[rov_id] = [float(formasyon_merkez[0]), float(formasyon_merkez[1])]
            
            return formasyon_pozisyonlar
            
        except Exception as e:
            # Fallback: merkez pozisyon hepsine
            fallback = {}
            for rov_id in grup_rovlar:
                fallback[rov_id] = [float(formasyon_merkez[0]), float(formasyon_merkez[1])]
            return fallback
    
    def get_hull_information(self, sample_count=50, g_id=0, kayit=False, hull_output=None, sessiz=True, offset_threshold=25.0):
        """
        🎯 Kapsamlı hull bilgisi: hull merkezi, sampled points, formasyon & grup detayları
        
        Bu fonksiyon:
        1. Hull'dan sample_count kadar örnek nokta alır (default 50)
        2. Formasyon bilgisini çeker (lider ROV ve formasyon parametreleri)
        3. Lider ROV'un grup_id'sinden tüm grup ROV'ların detaylarını alır
        4. Tüm bilgileri JSON-serializable format'ta döner
        
        🔹 Hull merkez kontrolü: Eski merkez ile yenisi arasında offset_threshold'dan az mesafe varsa hiçbir şey yapmaz (None döner)
        
        Args:
            sample_count: İstenen örnek nokta sayısı (default 50)
            g_id: Grup ID'si (default 0)
            kayit: True ise sonucu JSON dosyasına kaydet (append mode)
            hull_output: Özel hull dict (None ise otomatik calc)
            sessiz: True ise verbose log yazma
            offset_threshold: Hull merkez değişim eşiği (metres) - altında ise hiçbir şey yapma (default 25m)
        
        Returns:
            Dict:
            {
                'hull_center': (x, y),
                'hull_samples': [[x,y], [x,y], ...],  # sample_count kadar nokta
                'sample_count': N,
                'formasyon_id': 'LINE'|'CIRCLE'|vb,
                'formasyon_aralik': 10.5,
                'formasyon_merkez': (x, y),
                'formasyon_yaw': 45.0,
                'lider_rov_id': 0,
                'lider_yaw': 90.0,
                'grup_id': 0,
                'grup_bilgisi': {...},
                'formasyon_rov_pozisyonlari': {0: [100.5, 200.2], 1: [105.3, 198.1], ...},  # 🔹 ROV'ların formasyondaki hedef pozisyonları
                'ada_ve_engel_noktalar': [[x,y], [x,y], ...],  # 🔹 Dinamik engeller + statik ada cevresi noktaları
                'engel_bulutu_3d': [[x, z, y, kaynak], ...],   # 🔹 Ham engel bulutu (3D + kaynak)
                'timestamp': '2025-02-19 10:30:45'
            }
        """
        try:
            # 1️⃣ Hull ve sampled points
            # Önce formasyon cache'sini kontrol et (hızlı yol)
            formasyon_result_raw = self.filo.helper.get_formasyon_result(clear=False)
            formasyon_result = formasyon_result_raw.get('sonuc') if formasyon_result_raw else None
            
            # Formasyon cache'den hull bilgisi var mı?
            hull_info_from_formasyon = formasyon_result.get('hull_information', {}) if formasyon_result else {}
            
            if hull_info_from_formasyon and 'hull' in hull_info_from_formasyon:
                # ✅ Cache'den hull al (hızlı)
                hull_output = hull_info_from_formasyon.get('hull_data', {})
                hull_center = hull_info_from_formasyon.get('center', (0, 0))
            else:
                # ❌ Cache'de yok, yeni hull hesapla (yavaş)
                hulls_sınırları = self.filo.ada_cevre(sessiz=sessiz)
                hull_output = self.filo.helper.yeni_hull(hulls_sınırları, g_id=g_id, sessiz=sessiz)
                
                if not hull_output:
                    if not sessiz:
                        print("❌ Hull calculation başarısız")
                        print("⚠️ get_hull_information: result None")
                    return None
                
                hull_center = hull_output.get('center', (0, 0))
            
            # 🔹 HULL MERKEZ KONTROL - Önceki merkez ile offset karşılaştırması
            # Sadece memory-based cache kullan (dosya okuma yapma)
            import math
            
            if self.last_hull_center is not None:
                # Önceki merkez varsa, mesafe kontrol et
                distance = math.sqrt(
                    (hull_center[0] - self.last_hull_center[0])**2 + 
                    (hull_center[1] - self.last_hull_center[1])**2
                )
                
                if distance < offset_threshold:
                    # Hull merkez az değişti - KAYDETME!
                    if not sessiz:
                        print(f"⊙ Hull merkez az değişti ({distance:.2f}m < {offset_threshold}m) - Veri kaydedilmedi")
                        print("⚠️ get_hull_information: result None")
                    return None
            
            # Hull'dan sample'lar al
            hull_samples_np = self.filo.helper.get_100_samples(hull_output, sample_count)
            if hull_samples_np is not None:
                hull_samples = hull_samples_np.tolist() if hasattr(hull_samples_np, 'tolist') else hull_samples_np
            else:
                hull_samples = []
            
            # 2️⃣ Formasyon bilgisi - CACHE'DEN AL (tekrar hesapla etme)
            # Cache'de formasyon sonucu zaten var, başka hesaplama yok
            formasyon_id = formasyon_result.get('f_id', 'UNKNOWN') if formasyon_result else 'UNKNOWN'
            formasyon_aralik = formasyon_result.get('aralik', 0) if formasyon_result else 0
            formasyon_merkez = formasyon_result.get('merkez', hull_center) if formasyon_result else hull_center
            formasyon_yaw = formasyon_result.get('yaw', 0) if formasyon_result else 0
            formasyon_info_from_formasyon = formasyon_result.get('formasyon_information', {}) if formasyon_result else {}
            
            # 3️⃣ Lider ROV'u bul (formasyon_sec'ten)
            lider_info = self.filo.helper.find_leader_info(sessiz=True, g_id=g_id)
            lider_rov_id = lider_info[0] if lider_info[0] is not None else None
            lider_yaw = lider_info[1][3] if lider_info[1] and len(lider_info[1]) > 3 else 0
            
            # 4️⃣ Lider ROV'un grup_id'sini bul ve grup bilgileri al
            grup_id_actual = g_id
            if lider_rov_id is not None:
                lider_rov = self.filo.find_rov_by_id(lider_rov_id)
                if lider_rov:
                    grup_id_actual = lider_rov.group_id
                    lider_yaw = float(lider_rov.rotation_y)
            
            # 5️⃣ Grup detaylarını al
            grup_bilgisi = self.grup_bilgisi_al(grup_id_actual)
            
            # 6️⃣ Ada ve Engel noktalarını (dinamik + statik) al
            ada_ve_engel_noktalar = self.filo.get_engel_ve_ada(sessiz=True)
            engel_bulutu_3d = []
            if self.filo.ortam_ref and hasattr(self.filo.ortam_ref, 'engel_bulutu'):
                engel_bulutu_3d = list(self.filo.ortam_ref.engel_bulutu)
            
            # 7️⃣ Formasyon içerisindeki ROV'ların hedef pozisyonlarını hesapla
            grup_rovlar_ids = [rov.get('rov_id') for rov in grup_bilgisi.get('rovlar', [])] if grup_bilgisi else []
            formasyon_rov_pozisyonlari = self.formasyon_rov_pozisyonlari_al(
                formasyon_id=formasyon_id,
                formasyon_merkez=formasyon_merkez,
                formasyon_yaw=formasyon_yaw,
                grup_rovlar=grup_rovlar_ids
            )
            
            # 8️⃣ JSON-serializable sonuç oluştur
            result = {
                'hull_center': [float(hull_center[0]), float(hull_center[1])],
                'hull_samples': hull_samples if isinstance(hull_samples, list) else [],
                'sample_count': sample_count,
                'formasyon_id': str(formasyon_id),
                'formasyon_aralik': float(formasyon_aralik),
                'formasyon_merkez': [float(formasyon_merkez[0]), float(formasyon_merkez[1])] if formasyon_merkez else [0, 0],
                'formasyon_yaw': float(formasyon_yaw),
                'lider_rov_id': int(lider_rov_id) if lider_rov_id is not None else None,
                'lider_yaw': float(lider_yaw),
                'grup_id': grup_id_actual,
                'grup_bilgisi': grup_bilgisi if grup_bilgisi else {},
                'formasyon_rov_pozisyonlari': formasyon_rov_pozisyonlari,  # 🔹 ROV hedef pozisyonları formasyonda
                'ada_ve_engel_noktalar': ada_ve_engel_noktalar if ada_ve_engel_noktalar else [],
                'engel_bulutu_3d': engel_bulutu_3d if engel_bulutu_3d else [],
                'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                # 🔹 FORMASYON VE HULL BİLGİLERİ (formasyon_sec_impl'den alınan yapılandırılmış veriler)
                'hull_information': hull_info_from_formasyon,          # Hull objesi + hull_data + offset + engeller
                'formasyon_information': formasyon_info_from_formasyon # Formasyon + pozisyonlar + lider + grup
            }
            
            # 🔹 CACHE'E KAYDET
            self.last_hull_information = result
            self.hull_information_timestamp = datetime.now()
            self.last_hull_center = hull_center  # 🔹 Önceki merkez güncelle (sonraki karşılaştırma için)

            if kayit:
                success = self.save_hull_information('hull_information.json', result, sessiz=sessiz)
                if not success:
                    print("⚠️ Hull information kaydedilemedi")
            
            return result
            
        except Exception as e:
            if not sessiz:
                print(f"❌ get_hull_information hatası: {e}")
                print("⚠️ get_hull_information: result None")
                import traceback
                traceback.print_exc()
            return None
    
    def get_hull_100_samples(self, hull_output=None, sample_count=100):
        """
        🎯 Hull'dan parametreli örnek al (direkt + cache)
        
        Kullanım (önerilen):
            samples = filo.hull_info_manager.get_hull_100_samples()  # Hesapla + cache + döndür
            print(len(samples))  # 100
        
        Args:
            hull_output: Özel hull dict (None ise otomatik calc)
            sample_count: Örnek sayısı (default 100)
        
        Returns:
            [[x1,y1], [x2,y2], ...] veya None
        """
        try:
            result = self.filo.helper.get_100_samples(hull_output, sample_count)
            if result is not None:
                # NumPy array'i list'e dönüştür
                result_list = result.tolist() if hasattr(result, 'tolist') else result
                return result_list
            else:
                return None
        except Exception as e:
            print(f"❌ get_hull_100_samples hatası: {e}")
            return None
    
    def save_hull_information(self, filename='hull_information.json', result=None, sessiz=True):
        """
        🔹 (YARDIMCİ) Hull information'ı JSON dosyasına kaydet (APPEND mode)
        
        Dosya yoksa: Yeni JSON array oluşturur ve veri ekler
        Dosya varsa: Mevcut verinin altına yeni veri ekler (append)
        
        Args:
            filename: Çıktı JSON dosyası
            result: Kaydedilecek dict (None ise cache'den alır)
            sessiz: True ise verbose log yazma (başarı/hata mutlaka yazılır)
        
        Returns:
            True/False
        """
        try:
            import os
            
            data_to_save = result if result is not None else self.last_hull_information
            if data_to_save is None:
                print("❌ Hull information kaydedilemiyor - veri yok")
                return False
            
            # 🔹 JSON-SERIALIZABLE TEMIZLEME
            # SahteHull object'leri ve numpy arrays'leri çıkar
            data_clean = dict(data_to_save)  # Shallow copy
            
            # hull_information'daki object'leri kaldır
            if 'hull_information' in data_clean:
                hull_info = data_clean['hull_information']
                data_clean['hull_information'] = {
                    'center': tuple(map(float, hull_info.get('center', (0, 0)))),
                    'offset': float(hull_info.get('offset', 50)),
                    'yasakli_noktalar': hull_info.get('yasakli_noktalar', [])
                }
            
            # formasyon_information'daki pozisyonlar'ı tuple'dan list'e çevir
            if 'formasyon_information' in data_clean:
                form_info = data_clean['formasyon_information']
                if 'pozisyonlar' in form_info and isinstance(form_info['pozisyonlar'], dict):
                    # Pozisyonları list'e çevir JSON compat için
                    form_info['pozisyonlar'] = {str(k): list(v) if isinstance(v, tuple) else v 
                                                 for k, v in form_info['pozisyonlar'].items()}
            
            # Dosya varsa, mevcut veriyi oku ve yeni veriyi ekle
            if os.path.exists(filename):
                try:
                    with open(filename, 'r', encoding='utf-8') as f:
                        existing_data = json.load(f)
                except (json.JSONDecodeError, ValueError):
                    # Dosya hatalı ise, yeni yapı oluştur
                    print(f"⚠️ Dosya hatalı JSON, yeni yapı oluşturuluyor...")
                    existing_data = None
            else:
                existing_data = None

            def _normalize_existing_data(raw_data):
                if raw_data is None:
                    return {}
                if isinstance(raw_data, dict):
                    return raw_data
                if not isinstance(raw_data, list):
                    raw_data = [raw_data]
                if raw_data:
                    return {"Ortam-0": raw_data}
                return {}

            def _extract_ortam_index(key):
                if not isinstance(key, str):
                    return None
                if not key.startswith("Ortam-"):
                    return None
                try:
                    return int(key.split("-", 1)[1])
                except (ValueError, IndexError):
                    return None

            ortam_map = _normalize_existing_data(existing_data)

            if self._ortam_id is None:
                if ortam_map:
                    indices = [i for i in (_extract_ortam_index(k) for k in ortam_map.keys()) if i is not None]
                    next_idx = max(indices) + 1 if indices else 0
                    self._ortam_id = f"Ortam-{next_idx}"
                else:
                    self._ortam_id = "Ortam-0"

            data_clean['ortam_id'] = self._ortam_id

            if self._ortam_id not in ortam_map:
                ortam_map[self._ortam_id] = []
            ortam_map[self._ortam_id].append(data_clean)

            data_to_write = ortam_map
            
            # Dosyaya yaz
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(data_to_write, f, indent=2, ensure_ascii=False)
            
            # Kaç adet veri olduğunu göster (sessiz mode'da yazma)
            if not sessiz:
                if isinstance(data_to_write, dict):
                    data_count = len(data_to_write.get(self._ortam_id, []))
                else:
                    data_count = len(data_to_write) if isinstance(data_to_write, list) else 1
                print(f"✅ Hull information '{filename}' dosyasına eklendi ({data_count} tane kayıt var)")
            return True
            
        except Exception as e:
            print(f"❌ JSON yazma hatası: {e}")
            import traceback
            traceback.print_exc()
            return False


__all__ = ['HullInformationManager']
