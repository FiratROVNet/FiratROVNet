import json
from pathlib import Path

import numpy as np

from .replay_buffer import ReplayBuffer
from .sac_agent import SACAgent


class SAC:
    """
    Sualtı aracı (ROV) roll ve pitch stabilizasyonu için
    Soft Actor-Critic (SAC) çevre (Environment) yöneticisi.
    """
    
    LOSS_METRICS = ("actor_loss", "critic_loss")
    DEFAULT_METRICS = (
        "reward", "episode_reward", "best_episode_reward", 
        "buffer", "actor_loss", "critic_loss"
    )

    def __init__(self, filo_ref):
        self.filo = filo_ref
        
        # --- Model ve Hiperparametreler ---
        self.state_dim = 6         #[pitch, roll, p_rate, r_rate, prev_roll_act, prev_pitch_act]
        self.action_dim = 2        #[roll_power, pitch_power]
        self.action_scale = 0.15   # Kübik kontrol ve jitter engelleme için düşük güç çarpanı
        self.batch_size = 256
        self.warmup_steps = 1_000
        self.max_episode_steps = 600
        self.replay_size = 250_000
        self.device = "cpu"
        
        # --- RL Ajanı ve Hafıza ---
        self.agent = None
        self.replay_buffer = None
        self.training_enabled = True
        self.transfer_training_active = False
        
        # --- Eğitim Geçmişi ve Metrikler ---
        self.episode_count = 0
        self.episode_reward = 0.0
        self.best_episode_reward = float("-inf")
        self.total_updates = 0
        
        # --- ROV Durum Takipleri ---
        self.aktif_canli_egitim_rov_id = None
        self.canli_egitim_rov_ids = set()
        self._episode_steps = {}
        self._last_states = {}
        
        # Sarsıntı (Jerk) ve Sensör Gürültüsü Filtresi (EMA) İçin Hafızalar
        self._prev_actions = {}    
        self._previous_angles = {} 
        self._filtered_rates = {}  
        
        self._metric_history = {}
        self._loss_metric_history = {}
        self._last_loss_metrics = {}
        self._done_since_last_step = set()

        # --- Dosya Kayıt Yolları ---
        self.checkpoint_dir = Path(__file__).resolve().parent / "checkpoints"
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        self.best_model_path = self.checkpoint_dir / "sac_roll_pitch_best.pt"
        self.latest_model_path = self.checkpoint_dir / "sac_roll_pitch_latest.pt"
        self.metadata_path = self.checkpoint_dir / "sac_roll_pitch_meta.json"

    # =========================================================================
    # 1. TEMEL EĞİTİM DÖNGÜSÜ (CORE LOOP)
    # =========================================================================

    def configure_training(self, device="cpu", checkpoint_dir=None, **agent_kwargs):
        """SAC Ajanını ve Hafızayı başlatır, eski ağırlıkları yükler."""
        self.device = device
        
        if checkpoint_dir:
            self.checkpoint_dir = Path(checkpoint_dir)
            self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
            self.best_model_path = self.checkpoint_dir / "sac_roll_pitch_best.pt"
            self.latest_model_path = self.checkpoint_dir / "sac_roll_pitch_latest.pt"
            self.metadata_path = self.checkpoint_dir / "sac_roll_pitch_meta.json"

        self.agent = SACAgent(self.state_dim, self.action_dim, device=device, **agent_kwargs)
        self.replay_buffer = ReplayBuffer(self.state_dim, self.action_dim, self.replay_size, device=device)
        self.transfer_training_active = False
        
        self._durumu_yukle()
        return self

    def reset(self, rov_id=0):
        """Bölüm (Episode) başlarken ROV'un ortam değişkenlerini sıfırlar."""
        if isinstance(rov_id, (list, tuple, set)):
            rov_ids = [int(r) for r in rov_id if r is not None]
            return {rid: self._reset_rov_state(rid) for rid in rov_ids} if len(rov_ids) > 1 else self.reset(rov_ids[0])

        rid = int(rov_id)
        if self.aktif_canli_egitim_rov_id != rid:
            self._yeni_rov_icin_verileri_sifirla()
        
        self.aktif_canli_egitim_rov_id = rid
        self.episode_reward = 0.0
        return self._reset_rov_state(rid)

    def step(self, rov_id=0, action=None, train: bool | None = None):
        """Her simülasyon adımında çalışacak karar ve öğrenme mekanizması."""
        rid = int(rov_id)
        if self.agent is None or self.replay_buffer is None:
            self.configure_training()
            
        if rid in self._done_since_last_step:
            self.episode_reward = 0.0
            self._done_since_last_step.discard(rid)
            
        if rid not in self._last_states:
            self.reset(rid)

        state = self._last_states[rid]

        # Aksiyon Seçimi
        if action is None:
            is_warmup = len(self.replay_buffer) < self.warmup_steps and not self.transfer_training_active
            action = np.random.uniform(-1.0, 1.0, size=self.action_dim) if is_warmup else self.agent.select_action(state)
            
        action = np.clip(np.asarray(action, dtype=np.float32), -1.0, 1.0)

        # Aksiyonu Uygula ve Yeni Durumu Al
        self._motorlara_kubik_guc_ver(rid, action)
        next_state = self.get_state(rid)
        
        # Dalgalanma (Sarsıntı) Hesabı ve Ödül
        prev_act = self._prev_actions.get(rid, np.zeros(self.action_dim))
        action_diff = np.mean(np.square(action - prev_act))
        reward, reward_parts = self._reward_hesapla(next_state, action, action_diff)

        # Durumları Güncelle
        self._prev_actions[rid] = action.copy()
        self._episode_steps[rid] = self._episode_steps.get(rid, 0) + 1
        
        done = self._episode_bitti_mi(next_state, self._episode_steps[rid])
        self.episode_reward += reward
        tamamlanan_reward = self.episode_reward

        # Hafızaya Ekle ve Öğren (Eğitim)
        self.replay_buffer.add(state, action, reward, next_state, done)
        
        update_info = None
        should_train = self.training_enabled if train is None else bool(train)
        min_buffer_size = self.batch_size if self.transfer_training_active else max(self.batch_size, self.warmup_steps)
        
        if should_train and len(self.replay_buffer) >= min_buffer_size:
            update_info = self.agent.update(self.replay_buffer, self.batch_size)
            self.total_updates += 1
            self._loss_metriklerini_kaydet(rid, {
                "actor_loss": update_info["actor_loss"],
                "critic_loss": 0.5 * (update_info["critic1_loss"] + update_info["critic2_loss"])
            })

        # Bölüm Bitimi (Done) Yönetimi
        self._last_states[rid] = self._reset_rov_state(rid) if done else next_state
        if done:
            self.episode_count += 1
            self._done_since_last_step.add(rid)
            self._en_iyi_modeli_kaydet(rid, tamamlanan_reward)
            self._durumu_kaydet()

        # Metrikleri Raporla
        metrics = {
            "reward": reward,
            "episode_reward": self.episode_reward,
            "best_episode_reward": self.best_episode_reward if np.isfinite(self.best_episode_reward) else 0.0,
            "buffer": len(self.replay_buffer),
            **reward_parts,
            "actor_loss": self._last_loss_degeri_al(rid, "actor_loss"),
            "critic_loss": self._last_loss_degeri_al(rid, "critic_loss"),
        }
        if update_info:
            metrics["alpha"] = update_info["alpha"]
            
        self._metrikleri_kaydet(rid, metrics)
        return next_state, reward, done, {"metrics": metrics, "update": update_info, "action": action.copy()}

    # =========================================================================
    # 2. ÇEVRE (ENVIRONMENT) MATEMATİĞİ VE FİZİK
    # =========================================================================

    def get_state(self, rov_id=0):
        """Sensör verilerini 6 boyutlu State (Durum) uzayına çevirir."""
        rid = int(rov_id)
        pitch, roll = self._pitch_roll_derece_al(rid)
        pitch_rate, roll_rate = self._aci_hizi_derece_al(rid, pitch, roll)
        prev_act = self._prev_actions.get(rid, np.zeros(self.action_dim))

        return np.array([
            pitch / 45.0,        # Normalize Pitch
            roll / 45.0,         # Normalize Roll
            pitch_rate / 10.0,   # Normalize Pitch Açısal Hızı
            roll_rate / 10.0,    # Normalize Roll Açısal Hızı
            prev_act[0],         # Bir önceki Roll komutu
            prev_act[1],         # Bir önceki Pitch komutu
        ], dtype=np.float32)

    def _motorlara_kubik_guc_ver(self, rov_id, action):
        """Yapay zeka aksiyonlarını kübik olarak yumuşatır ve motorlara iletir."""
        rov = self.filo.find_rov_by_id(int(rov_id))
        if not rov or not getattr(rov, "motorlar", None) or len(rov.motorlar) < 8:
            return

        # KÜBİK KONTROL (Hassas Dengeleme Hilesi)
        action_roll = float(action[0]) ** 3
        action_pitch = float(action[1]) ** 3

        roll_gucu = action_roll * self.action_scale
        pitch_gucu = action_pitch * self.action_scale
        gucler =[float(getattr(motor, "guc", 0.0)) for motor in rov.motorlar]
        
        # Dikey motorlara (4,5,6,7) güç dağılımı
        dikey_komutlar = {
            4: roll_gucu + pitch_gucu,
            5: -roll_gucu + pitch_gucu,
            6: roll_gucu - pitch_gucu,
            7: -roll_gucu - pitch_gucu,
        }
        
        for idx, guc in dikey_komutlar.items():
            gucler[idx] = max(-1.0, min(1.0, guc))

        self.filo.motorlari_calistir(rov_id, gucler)

    def _reward_hesapla(self, state, action, action_diff):
        """ROV'un dengesine ve motor kullanımına göre ödül/ceza belirler."""
        pitch_deg = abs(float(state[0]) * 45.0)
        roll_deg = abs(float(state[1]) * 45.0)
        pitch_rate = abs(float(state[2]) * 10.0)
        roll_rate = abs(float(state[3]) * 10.0)

        # 1. Ölüm (Crash) Cezası
        if pitch_deg > 70.0 or roll_deg > 70.0:
            return -50.0, {"pitch_roll_odulu": 0.0, "jerk_penalty": 0.0}

        # 2. DOĞRUSAL (Linear) ÖDÜL KÖRLEŞMEYİ ENGELLER
        # Hata 70 dereceye yaklaştıkça ödül sıfırlanır, merkeze yaklaştıkça 1'e doğru artar
        pitch_reward = (1.0 - (pitch_deg / 30.0))
        roll_reward = (1.0 - (roll_deg / 30.0))

        # 3. Cezalar (Penalties)
        stability_penalty = 0.2 * (pitch_rate + roll_rate)       # Aşırı sallanma cezası
        power_penalty = 0.2 * np.mean(np.square(action))         # Kaba motor kullanımı
        jerk_penalty = 0.6 * action_diff                         # Titreme/Sarsıntı cezası

        reward = pitch_reward + roll_reward - stability_penalty - power_penalty - jerk_penalty

        return float(reward), {
            "pitch_roll_odulu": float(pitch_reward + roll_reward),
            "jerk_penalty": float(-jerk_penalty)
        }

    def _episode_bitti_mi(self, state, steps):
        """Maksimum süreye ulaşıldı mı veya araç devrildi mi?"""
        pitch_deg = abs(float(state[0]) * 45.0)
        roll_deg = abs(float(state[1]) * 45.0)
        return steps >= self.max_episode_steps or pitch_deg > 70.0 or roll_deg > 70.0

    # =========================================================================
    # 3. DOSYA YÖNETİMİ VE KAYIT (SAVE/LOAD)
    # =========================================================================

    def _durumu_yukle(self):
        """Meta verileri ve PyTorch model ağırlıklarını diskten yükler."""
        if self.metadata_path.exists():
            try:
                with open(self.metadata_path, "r") as f:
                    meta = json.load(f)
                    self.episode_count = meta.get("episode_count", 0)
                    self.best_episode_reward = meta.get("best_episode_reward", float("-inf"))
                    self.total_updates = meta.get("total_updates", 0)
                print(f"🔄 Eğitim yüklendi: Episode {self.episode_count}, En İyi Ödül {self.best_episode_reward:.2f}")
            except Exception:
                pass

        if self.agent:
            model_path = self.latest_model_path if self.latest_model_path.exists() else self.best_model_path
            if model_path.exists():
                try:
                    self.agent.load(model_path)
                    self.transfer_training_active = True
                    print(f"✅ Model ağırlıkları yüklendi: {model_path.name}")
                except Exception:
                    pass

    def _durumu_kaydet(self):
        """Mevcut bölüm sayılarını ve latest modeli kaydeder."""
        try:
            with open(self.metadata_path, "w") as f:
                json.dump({
                    "episode_count": self.episode_count,
                    "best_episode_reward": self.best_episode_reward,
                    "total_updates": self.total_updates
                }, f)
                
            if self.agent:
                self.agent.save(self.latest_model_path)
        except Exception:
            pass

    def _en_iyi_modeli_kaydet(self, rov_id, episode_reward):
        """O ana kadarki en iyi bölüm (episode) skoru elde edildiyse best.pt'ye yazar."""
        if self.agent and float(episode_reward) > self.best_episode_reward:
            self.best_episode_reward = float(episode_reward)
            try:
                self.agent.save(self.best_model_path)
                print(f"🏆 YENİ REKOR | ROV-{int(rov_id)} | Episode: {self.episode_count} | Ödül: {self.best_episode_reward:.1f}")
            except Exception:
                pass

    # =========================================================================
    # 4. YARDIMCI METOTLAR (HELPERS)
    # =========================================================================

    def _reset_rov_state(self, rid):
        self._prev_actions[rid] = np.zeros(self.action_dim, dtype=np.float32)
        self._filtered_rates[rid] = (0.0, 0.0) # Filtreyi sıfırla
        state = self.get_state(rid)
        self._last_states[rid] = state
        self._episode_steps[rid] = 0
        self._previous_angles[rid] = self._pitch_roll_derece_al(rid)
        self.canli_egitim_rov_ids.add(rid)
        return state

    def _yeni_rov_icin_verileri_sifirla(self):
        if self.replay_buffer:
            self.replay_buffer = ReplayBuffer(self.state_dim, self.action_dim, self.replay_size, device=self.device)
        self.transfer_training_active = (self.agent is not None and self.total_updates > 0)
        self._last_states.clear()
        self._episode_steps.clear()
        self._previous_angles.clear()
        self._prev_actions.clear()
        self._filtered_rates.clear() # Filtreyi sıfırla
        self._done_since_last_step.clear()

    def _pitch_roll_derece_al(self, rov_id):
        rov = self.filo.find_rov_by_id(int(rov_id))
        if not rov:
            return 0.0, 0.0
            
        gnc = getattr(rov, "gnc", None)
        pitch = getattr(gnc, "bullet_pitch", None) or getattr(rov, "rotation_x", 0.0)
        roll = getattr(gnc, "bullet_roll", None) or getattr(rov, "rotation_z", 0.0)
        
        return self._aci_normalize(pitch), self._aci_normalize(roll)

    def _aci_hizi_derece_al(self, rov_id, pitch, roll):
        """Sensör gürültüsünü engellemek için EMA filtreli açısal hız döndürür."""
        prev_p, prev_r = self._previous_angles.get(int(rov_id), (pitch, roll))
        self._previous_angles[int(rov_id)] = (pitch, roll)
        
        raw_p_rate = self._aci_farki(pitch, prev_p)
        raw_r_rate = self._aci_farki(roll, prev_r)
        
        # EMA Filtresi: Ani sıçramaları (jitter'ı) emer
        eski_filt = self._filtered_rates.get(int(rov_id), (0.0, 0.0))
        yeni_filt_p = 0.5 * eski_filt[0] + 0.5 * raw_p_rate
        yeni_filt_r = 0.5 * eski_filt[1] + 0.5 * raw_r_rate
        
        self._filtered_rates[int(rov_id)] = (yeni_filt_p, yeni_filt_r)
        
        return yeni_filt_p, yeni_filt_r

    @staticmethod
    def _aci_normalize(aci):
        return ((float(aci) + 180.0) % 360.0) - 180.0

    @staticmethod
    def _aci_farki(yeni, eski):
        return ((float(yeni) - float(eski) + 180.0) % 360.0) - 180.0

    def _metrikleri_kaydet(self, rov_id, metrics):
        history = self._metric_history.setdefault(int(rov_id), {})
        for name, value in metrics.items():
            if name not in self.LOSS_METRICS:
                vals = history.setdefault(name,[])
                vals.append(float(value))
                if len(vals) > 600: del vals[:-600]

    def _loss_metriklerini_kaydet(self, rov_id, metrics):
        history = self._loss_metric_history.setdefault(int(rov_id), {})
        last_vals = self._last_loss_metrics.setdefault(int(rov_id), {})
        for name, value in metrics.items():
            vals = history.setdefault(name, [])
            vals.append(float(value))
            last_vals[name] = float(value)
            if len(vals) > 5000: del vals[:-5000]

    def _last_loss_degeri_al(self, rov_id, metrik):
        return self._last_loss_metrics.get(int(rov_id), {}).get(metrik, 0.0)

    # --- Canlı Arayüz (Monitoring) Metotları ---
    def canli_egitim_metrikleri_al(self):
        return self.DEFAULT_METRICS

    def canli_egitim_rov_id_al(self, varsayilan_grup_id=None):
        return int(self.aktif_canli_egitim_rov_id) if self._egitime_uygun_mu(self.aktif_canli_egitim_rov_id) else None

    def canli_egitim_rovleri_al(self, varsayilan_grup_id=None):
        rid = self.canli_egitim_rov_id_al(varsayilan_grup_id)
        return [rid] if rid is not None else[]

    def canli_egitim_adimi(self, rov_id=None):
        rov_ids =[int(self.aktif_canli_egitim_rov_id)] if rov_id is None else self._rov_id_listesine_cevir(rov_id)[:1]
        return {rid: self.step(rov_id=rid) for rid in rov_ids if self._egitime_uygun_mu(rid)}

    def metrik_gecmisi(self, rov_id, metrik, limit: int = 240):
        history_dict = self._loss_metric_history if metrik in self.LOSS_METRICS else self._metric_history
        return history_dict.get(int(rov_id), {}).get(str(metrik), [])[-int(limit):]

    def _rov_id_listesine_cevir(self, rov_id):
        return[int(rid) for rid in rov_id if rid is not None] if isinstance(rov_id, (list, tuple, set)) else[int(rov_id)]

    def _egitime_uygun_mu(self, rov_id):
        try:
            rov = self.filo.find_rov_by_id(int(rov_id)) if rov_id is not None else None
            return bool(rov and not getattr(rov, "is_destroyed", False))
        except Exception:
            return False