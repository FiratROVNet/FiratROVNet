from __future__ import annotations

import copy
from pathlib import Path

import torch
import torch.nn.functional as F
from torch.optim import Adam

from .networks import Actor, Critic
import math

class SACAgent:
    def __init__(
        self,
        state_dim: int,
        action_dim: int,
        device: str = "cpu",
        gamma: float = 0.99,
        tau: float = 0.005,
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        alpha_lr: float = 3e-4,
        hidden_dim: int = 256,
        automatic_entropy_tuning: bool = True,
        initial_alpha: float = 0.15,
    ):
        self.device = device
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.tau = tau
        self.automatic_entropy_tuning = automatic_entropy_tuning

        self.actor = Actor(state_dim, action_dim, hidden_dim).to(device)
        self.critic1 = Critic(state_dim, action_dim, hidden_dim).to(device)
        self.critic2 = Critic(state_dim, action_dim, hidden_dim).to(device)
        self.target_critic1 = copy.deepcopy(self.critic1).to(device)
        self.target_critic2 = copy.deepcopy(self.critic2).to(device)

        self.actor_optimizer = Adam(self.actor.parameters(), lr=actor_lr)
        self.critic1_optimizer = Adam(self.critic1.parameters(), lr=critic_lr)
        self.critic2_optimizer = Adam(self.critic2.parameters(), lr=critic_lr)

        if automatic_entropy_tuning:
            self.target_entropy = -float(action_dim)

            self.log_alpha = torch.tensor(
                [math.log(float(initial_alpha))],
                requires_grad=True,
                device=device
            )

            self.alpha_optimizer = Adam([self.log_alpha], lr=alpha_lr)
            self.alpha = self.log_alpha.exp()
        else:
            self.log_alpha = None
            self.alpha_optimizer = None
            self.alpha = torch.tensor(float(initial_alpha), device=device)

    def select_action(self, state, evaluate: bool = False):
        state = torch.as_tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            action, _, deterministic_action = self.actor.sample(state)
        selected_action = deterministic_action if evaluate else action
        return selected_action.cpu().numpy()[0]

    def update(self, replay_buffer, batch_size: int):
        states, actions, rewards, next_states, dones = replay_buffer.sample(batch_size)

        with torch.no_grad():
            future_actions, future_log_probs, _ = self.actor.sample(next_states)
            q1_target = self.target_critic1(next_states, future_actions)
            q2_target = self.target_critic2(next_states, future_actions)
            min_q_target = torch.min(q1_target, q2_target)
            target_q = rewards + self.gamma * (1 - dones) * (min_q_target - self.alpha * future_log_probs)

        q1 = self.critic1(states, actions)
        q2 = self.critic2(states, actions)
        critic1_loss = F.mse_loss(q1, target_q)
        critic2_loss = F.mse_loss(q2, target_q)

        self.critic1_optimizer.zero_grad()
        critic1_loss.backward()
        self.critic1_optimizer.step()

        self.critic2_optimizer.zero_grad()
        critic2_loss.backward()
        self.critic2_optimizer.step()

        self._set_critic_grad(False)
        new_actions, log_probs, _ = self.actor.sample(states)
        q1_new = self.critic1(states, new_actions)
        q2_new = self.critic2(states, new_actions)
        actor_loss = (self.alpha * log_probs - torch.min(q1_new, q2_new)).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()
        self._set_critic_grad(True)

        if self.automatic_entropy_tuning:
            alpha_loss = -(self.log_alpha * (log_probs + self.target_entropy).detach()).mean()
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            self.alpha = self.log_alpha.exp()
        else:
            alpha_loss = torch.tensor(0.0, device=self.device)

        self.soft_update(self.critic1, self.target_critic1)
        self.soft_update(self.critic2, self.target_critic2)

        return {
            "critic1_loss": critic1_loss.item(),
            "critic2_loss": critic2_loss.item(),
            "actor_loss": actor_loss.item(),
            "alpha_loss": alpha_loss.item(),
            "alpha": self.alpha.item(),
            "q1_mean": q1.mean().item(),
            "q2_mean": q2.mean().item(),
            "target_q_mean": target_q.mean().item(),
        }

    def soft_update(self, source_net, target_net):
        for source_param, target_param in zip(source_net.parameters(), target_net.parameters()):
            target_param.data.copy_(self.tau * source_param.data + (1 - self.tau) * target_param.data)

    def save(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "actor": self.actor.state_dict(),
            "critic1": self.critic1.state_dict(),
            "critic2": self.critic2.state_dict(),
            "target_critic1": self.target_critic1.state_dict(),
            "target_critic2": self.target_critic2.state_dict(),
            "actor_optimizer": self.actor_optimizer.state_dict(),
            "critic1_optimizer": self.critic1_optimizer.state_dict(),
            "critic2_optimizer": self.critic2_optimizer.state_dict(),
            "alpha": self.alpha.detach().cpu(),
            "automatic_entropy_tuning": self.automatic_entropy_tuning,
        }
        if self.automatic_entropy_tuning:
            payload["log_alpha"] = self.log_alpha.detach().cpu()
            payload["alpha_optimizer"] = self.alpha_optimizer.state_dict()
        torch.save(payload, path)

    def load(self, path, map_location=None):
        payload = torch.load(path, map_location=map_location or self.device)
        self.actor.load_state_dict(payload["actor"])
        self.critic1.load_state_dict(payload["critic1"])
        self.critic2.load_state_dict(payload["critic2"])
        self.target_critic1.load_state_dict(payload["target_critic1"])
        self.target_critic2.load_state_dict(payload["target_critic2"])
        self.actor_optimizer.load_state_dict(payload["actor_optimizer"])
        self.critic1_optimizer.load_state_dict(payload["critic1_optimizer"])
        self.critic2_optimizer.load_state_dict(payload["critic2_optimizer"])
        if self.automatic_entropy_tuning and "log_alpha" in payload:
            self.log_alpha.data.copy_(payload["log_alpha"].to(self.device))
            self.alpha_optimizer.load_state_dict(payload["alpha_optimizer"])
            self.alpha = self.log_alpha.exp()

    def _set_critic_grad(self, enabled: bool):
        for param in self.critic1.parameters():
            param.requires_grad = enabled
        for param in self.critic2.parameters():
            param.requires_grad = enabled
