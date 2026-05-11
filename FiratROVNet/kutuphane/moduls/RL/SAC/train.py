from __future__ import annotations


def train_filo_sac(filo, rov_id: int = 0, total_steps: int = 500_000, log_interval: int = 1000):
    """Mevcut Filo nesnesi uzerinden SAC egitim dongusu calistirir."""

    sac = getattr(filo, "sac", None) or getattr(filo, "SAC", None)
    if sac is None:
        raise RuntimeError("Filo uzerinde SAC modulu bulunamadi.")
    if sac.agent is None or sac.replay_buffer is None:
        sac.configure_training()

    sac.reset(rov_id)
    last_info = None
    for step in range(1, total_steps + 1):
        _next_state, reward, done, info = sac.step(rov_id=rov_id)
        last_info = info.get("update")
        if done:
            print(f"Episode {sac.episode_count} | Reward: {sac.episode_reward:.2f}")
        if last_info and step % log_interval == 0:
            print(
                f"Step: {step} | Buffer: {len(sac.replay_buffer)} | "
                f"Reward: {reward:.3f} | Actor Loss: {last_info['actor_loss']:.4f} | "
                f"Critic1 Loss: {last_info['critic1_loss']:.4f} | Alpha: {last_info['alpha']:.4f}"
            )
