# Fırat-GNC AI Copilot Instructions

## Project Overview

**Fırat-GNC** is an AI-driven multi-agent underwater ROV/AUV swarm simulation and control system developed at Fırat University. It combines:
- **Physics-based 3D simulation** (Ursina engine with Panda3D)
- **Graph Attention Networks (GAT)** for distributed AI decision-making
- **Reinforcement Learning (RL/PPO)** for adaptive swarm behaviors
- **Realistic acoustic modem simulation** with packet loss, delay, and noise
- **Live console interaction** for real-time parameter tuning

## Architecture

### Core Components

**Simulation Engine** ([FiratROVNet/simulasyon.py](FiratROVNet/simulasyon.py#L1))
- `Ortam` class: Main 3D environment, physics simulation, island/obstacle generation
- `ROV` class: Individual vehicle entity with physics, sensors, and animation
- Uses Ursina (game engine wrapper around Panda3D) for rendering

**Guidance, Navigation & Control (GNC)** ([FiratROVNet/gnc.py](FiratROVNet/gnc.py#L1))
- `Filo` class (3000+ lines): Main swarm coordinator; manages multi-ROV formations, path planning, obstacle avoidance
- `TemelGNC`: Per-ROV controller for APF (Artificial Potential Fields) and movement
- `Koordinator`: Bridges simulation (X:right, Y:forward, Z:depth) ↔ Ursina (X:right, Y:up, Z:forward) coordinate systems
- **Critical**: All GNC limits tied to `GATLimitleri` config class (e.g., CARPISMA=10m, ENGEL=20m, KOPMA=40m)

**AI Brain** ([GAT/](GAT/))
- `FiratAnalizci` (gat_test.py): Graph Attention Network that processes neighborhood information
- Classifies 6 critical codes locally: 0=OK, 1=Obstacle, 2=Collision risk, 3=Link break, 4=Position lost, 5=Far from leader
- Per-ROV distributed decision-making; no central controller

**Communication** ([FiratROVNet/iletisim.py](FiratROVNet/iletisim.py))
- `AkustikModem`: Simulates underwater acoustic modem with delay, packet loss, SNR/noise
- Each ROV broadcasts sensor data; neighbors receive based on `ModemAyarlari.iletisim_menzili` range

**RL/PPO Integration** ([RL_PPO/](RL_PPO/))
- **12 hybrid models** (2 per task): RL (Q-learning/DQN) + PPO (Actor-Critic) versions
- Tasks: A* pathfinding, convex hull safety zones, leader selection, path following, formation selection
- Pattern: RL/PPO decides action; ~50% of the time delegates to original FiratROVNet function with fallback
- See [ENTEGRASYON_OZETI.md](ENTEGRASYON_OZETI.md) for detailed integration architecture

### Configuration & Constants

**Config** ([FiratROVNet/config.py](FiratROVNet/config.py#L1))
- `GATLimitleri`: Critical thresholds (CARPISMA, ENGEL, KOPMA, UZAK) - **used by GAT training, sensor range config, and GNC logic**
- `SensorAyarlari`: Per-ROV sensor ranges (obstacle detection, communication range, collision avoidance distance)
- `HareketAyarlari`: Formation constraints, pool boundaries, convex hull offsets
- **Constraint**: Sensor ranges MUST align with `GATLimitleri` thresholds or behavior diverges between training/simulation

**Helper Math Library** ([FiratROVNet/kutuphane/helper/gnc_helper.py](FiratROVNet/kutuphane/helper/gnc_helper.py#L1))
- `Hidrodinamik`: Water physics (drag, buoyancy, thrust dynamics) — updated motor thrust and drag coefficients
- `BasitKalmanFiltresi`: 1D Kalman filtering for smooth velocity/position updates (R=0.5, Q=0.01)
- `FiloHelper`: Complex GNC calculations — **NEW**: threadsafe `engel_bul()` with raycast + cache mode, improved `vektor()` method, `apf()` vectorial calculations, waypoint handling
- `TemelGNCHelper`: Per-ROV physics and APF application — **NEW**: `vektor_to_motor_sim()` for direct physics application
- Imports: Shapely (polygon geometry), alphashape (concave hulls), scipy.ConvexHull (fallback)

## Key Workflows

### Running Simulation
```bash
python main.py
# Starts 6-ROV swarm with 15 obstacles, loads pre-trained GAT model, opens Minimap + Matplotlib Harita
# Press `i` → tab → enter for live Python console (e.g., `git(0, 100, 50)` to move ROV-0 to target)
# Console commands: git(), move(), get(), set(), Ada(), ROV(), debug, filo, rovs, cfg, harita

# Examples:
filo.git(0, 100, 50, 0, ai=True)          # Move ROV-0 to (100, 50, 0) with AI
debug.apf(0)                               # Show APF vectors for ROV-0
app.harita.goster(True, convex=True)      # Show Matplotlib map with convex hull
filo.formasyon_sec(dinamik=True)          # Auto-select best formation
```

### Training GAT Model
```bash
# From GAT/ folder; generates `rov_modeli_multi.pth`
python GAT/gat_train.py --epochs 5000 --hidden_channels 16 --num_heads 4
```

### Testing
```bash
# Headless CI test (no GPU/display required)
python run_tests.py
```

### Live Console Commands (during simulation)
- `git(rov_id, x, y, z=None, ai=True)`: Set target waypoint for ROV (Simulation coordinates: X:right, Y:forward, Z:depth)
- `move(rov_id, direction, force)`: Direct motor command (ileri/geri/sağ/sol/çık/bat/dur)
- `get(rov_id, data_type)`: Query ROV state (position, velocity, battery, yaw, role, sensor, sonar, lidar)
- `set(rov_id, param, value)`: Tune runtime parameters (sensor_config, rol, yaw)
- `Ada(island_id, x, y)`: Relocate obstacle island dynamically
- `ROV(rov_id, x, y, z)`: Get/set ROV position (returns (x, y, z) if position not specified)
- `debug.apf(rov_id)`: Show APF vector calculation for ROV (hedef, engeller, rovlar)
- `debug.vektor(rov_id_ilk, rov_id_ikinci)`: Draw vector between two ROVs on minimap
- `filo.formasyon_sec(dinamik=True)`: Auto-select best formation using convex hull
- `filo.minimap(scale=1.0)`: Show/hide HUD minimap (Ursina UI)

## Critical Patterns & Conventions

### Coordinate System Transitions
Always use `Koordinator.sim_to_ursina()` and `ursina_to_sim()` for position conversions. **Failure to translate = invisible or out-of-bounds vehicles.** Test in both coordinate systems when debugging rendering issues.

### GNC Limits Consistency
All three systems must agree on thresholds:
1. **GAT training data generation** — uses `GATLimitleri` thresholds to label training examples (e.g., distance < 10m = Code 2 collision risk)
2. **Sensor configuration** — `SensorAyarlari` must set `engel_mesafesi = GATLimitleri.ENGEL` to trigger correct GAT codes during runtime
3. **GNC logic** — formation, APF, and threat-response functions read same `GATLimitleri.CARPISMA` / `ENGEL` / `KOPMA` limits

**Pattern violation symptom**: Simulation exhibits different swarm behavior than training environment.

### Distributed vs. Centralized Decision-Making
- **Never add a global controller**: Each `TemelGNC` (per-ROV) runs APF independently; `Filo` only aggregates for minimap/formation templates
- **GAT codes are local broadcasts**: If ROV detects obstacle (Code 1), it flags neighbors but doesn't request permission
- **Formation is soft constraint**: ROVs attract to formation positions; other behaviors (collision avoidance, path following) override

### Error Handling & Fallback
- Shapely/alphashape import failures disable contour visualization but don't crash simulation
- Original GNC functions used in RL/PPO fallback pattern (graceful degradation if RL model fails)
- Physics calculations defend against zero-division and NaN propagation via `BasitKalmanFiltresi` smoothing

### File Organization
```
FiratROVNet/
  ├─ simulasyon.py      → Ursina rendering, ROV/island/obstacle entities
  ├─ gnc.py             → Filo + TemelGNC core logic (1800+ lines)
  ├─ config.py          → All tunable constants
  ├─ iletisim.py        → Modem simulation
  ├─ hull.py            → Convex/concave hull wrapping
  └─ kutuphane/helper/
      ├─ gnc_helper.py  → Math & geometry (3400+ lines)
      └─ simulasyon_helper.py → Coordinate conversion, grid helpers

GAT/
  ├─ gat_train.py       → Training loop with configurable hyperparams
  ├─ gat_test.py        → FiratAnalizci inference class
  └─ gat_graf.py        → Data preprocessing pipeline

RL_PPO/
  ├─ a_star/, convex_hull/, lider_sec/, git_path/, formasyon*/
  │   └─ *_rl.py, *_ppo.py → Hybrid RL+original function wrappers
  └─ RL_PPO_MODELS_DOCUMENTATION.py → Full model catalog
```

## New Systems & Recent Enhancements (February 2026)

### Thread-Safe Console Access
- `engel_bul()` now supports console thread via cache mode: raycast in main thread, cache in console thread
- `_is_main_thread()` checks for safe Ursina/Panda3D operations
- Command queue for thread-safe operations from console

### Improved Vector & APF System
- `vektor()` method refactored: supports 2D minimap drawing with keyword arguments (`rov_id_ilk`, `rov_id_ikinci`, `vektor`, `renk`, `uzunluk`, `reverse`)
- `apf()` now returns dict with hedef/engeller/rovs sub-dictionaries for modular processing
- `apf_temizle()` clears old vector visualization before recalculating

### Hydrodynamic & Physics (TemelGNCHelper)
- `vektor_to_motor_sim()`: Direct physics application (Sim coordinate system → motor commands)
- Kalman filtering per-axis: X, Y, Z smoothing (R=0.5, Q=0.01) for stable velocity estimates
- `BasitKalmanFiltresi`: 1D Kalman filters reduce jitter and improve formation stability

### Minimap & Visualization Enhancements
- `Minimap`: GPU-rendered HUD radar (Ursina UI) — 0 FPS overhead vs matplotlib
- `Harita`: Matplotlib-based 2D map with GPS pins, islands, engel_bulutu cloud, A* path visualization
- Dual system: Real-time HUD (Minimap) + Separate window map (Harita)
- `engel_bulutu`: Cloud of detected obstacle points updated from ROV sonar/lidar

### Formation Selection (Dinamik)
- `formasyon_sec()`: Convex hull-based formation selection with thread-safe mode
- Yaw synchronization (optional): `yaw_senkronizasyon_mesafesi`, `maksimum_yaw_donme_hizi`
- Returns (formasyon_id, aralik, yaw, merkez_koordinat) tuple

### Motor Thrust Physics
- `MAX_ITME_KUVVETI`: Increased to 100N (from 50N) for stronger swarm control
- `DRAG_KATSAYISI_CD`: 0.9 (from 0.8) for more realistic underwater resistance
- Batarya system integrated with velocity: battery depletes faster at high speeds

### New ROV Methods
- `ROV.ekle()`: Dynamically add ROV to simulation at specific coordinates
- `ROV.cikar()`: Safely remove ROV and renumber remaining ROV IDs
- `ROV.get()` / `ROV.set()`: Direct sensor/parameter access (bypasses GNC)
- `ROV.move()`: Manual motor commands independent of GNC

### Sensor Caching System
- `son_sonar_mesafesi`, `son_lidar_mesafeleri` (dict): Cache values from last main-thread raycast
- Console can read cache without triggering raycast (thread-safe)
- `_engel_bul_cache_sonuc()`: Rebuild engel listesi from cached sensor values

### Adding a New GNC Behavior
1. Add threshold constant to `GATLimitleri` in [config.py](FiratROVNet/config.py)
2. Add training data labeling rule in GAT dataset generation (if training new GAT)
3. Implement behavior in `TemelGNC` class in [gnc.py](FiratROVNet/gnc.py), calling `self.helper.apf_temizle()` for vector cleanup
4. Register debug function in `Debug` class if interactive testing needed

### Tuning Swarm Formation
Edit `HareketAyarlari.FORMASYON_*` in [config.py](FiratROVNet/config.py) — don't modify formation logic directly in [gnc.py](FiratROVNet/gnc.py); changes are overridden at runtime from config.

### Extending Communication Model
Modify `ModemAyarlari` in [config.py](FiratROVNet/config.py) and `AkustikModem.simulate_transmission()` in [iletisim.py](FiratROVNet/iletisim.py). Verify delay/loss model in unit tests ([run_tests.py](run_tests.py)).

## Dependencies & Notes

- **PyTorch 2.0+** required for GAT (torch-geometric)
- **Ursina 5.x–6.x** (7.x breaks OBJ mesh loading); Panda3D backend
- **Shapely 2.0+** for polygon operations (critical for convex hull fallback)
- **No GPU required** for simulation (CPU-only mode works); GPU optional for GAT training
- **Thread-safe**: Communication modem and GNC helper use locks; safe to call from console

## Testing Strategy

- Unit tests in [run_tests.py](run_tests.py) run headless (no display/GPU) — validates core math
- Integration tests combine GAT + simulation — verify codes match expected behaviors
- Regression tests track convex hull accuracy and formation geometry
- Console interaction ([test.py](test.py)) validates Python integration

---

**Last Updated**: February 2026  
**Contact**: Fırat University Autonomous Systems & AI Lab
