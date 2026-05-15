from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODELS_AI_DIR = PROJECT_ROOT / "Models-AI"

GAT_MODEL = MODELS_AI_DIR / "GAT" / "rov_modeli_multi.pth"
YOLOV8N_MODEL = MODELS_AI_DIR / "YOLO" / "yolov8n.pt"
SAC_ROLL_PITCH_DIR = MODELS_AI_DIR / "SAC" / "roll_pitch"


def path_str(path: Path) -> str:
    return str(path)
