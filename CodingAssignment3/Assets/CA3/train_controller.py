import json
from pathlib import Path
import time
import cma
import numpy as np
import torch
from torch import nn

ROOT = Path(__file__).resolve().parent / "Data"
STATE = 12  # UR5e: 6 joint angles (q) + 6 joint velocities (qd)
ACTION = 6  # UR5e: 6 position-servo targets (q*)
FEAT = STATE
N_PARAMS = ACTION * FEAT  # no bias: b = 0 keeps q = goal as the resting point

# UR5e "home" keyframe from the Menagerie MJCF.
HOME = np.array([-np.pi / 2, -np.pi / 2, np.pi / 2,
                 -np.pi / 2, -np.pi / 2, 0.0], dtype=np.float32)
# UR5e joint ranges from the MJCF: ±2π everywhere except the elbow (±π).
JOINT_LO = np.array([-2 * np.pi, -2 * np.pi, -np.pi,
                     -2 * np.pi, -2 * np.pi, -2 * np.pi], dtype=np.float32)
JOINT_HI = np.array([2 * np.pi, 2 * np.pi, np.pi,
                     2 * np.pi, 2 * np.pi, 2 * np.pi], dtype=np.float32)
RANDOM_SCALE = 0.35

VEL_PENALTY = 0.05
ACTION_SMOOTH_PENALTY = 0.1
ACTION_SMOOTH = 0.1  # matches WorldModelController.actionSmoothing default
MAX_DELTA = 0.05
HORIZON = 200
EPISODES = 16
CMA_ITERS = 60


class DynamicsModel(nn.Module):
    """Same architecture as train.py: MLP predicting the normalized
    next-state delta from [q, qd, q*]."""

    def __init__(self, state=STATE, action=ACTION, hidden=256, depth=3):
        super().__init__()
        layers = []
        width = state + action
        for _ in range(depth):
            layers += [nn.Linear(width, hidden), nn.ReLU()]
            width = hidden
        layers.append(nn.Linear(width, state))
        self.net = nn.Sequential(*layers)

    def forward(self, x):
        return self.net(x)


def build_dynamics():
    model = DynamicsModel()
    path = ROOT / "dynamics.pt"
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}. Run train.py first.")
    model.load_state_dict(torch.load(path, map_location="cpu", weights_only=True))
    model.eval()
    return model


def load_norm():
    path = ROOT / "norm.npz"
    if not path.exists():
        raise FileNotFoundError(f"Missing {path}. Run train.py first.")
    data = np.load(path)
    return data["x_mean"], data["x_std"], data["y_mean"], data["y_std"]


def unpack(theta):
    # b is fixed at 0. With action = goal + W @ [q - goal, qd], a resting
    # arm (qd = 0) is only in equilibrium at q = goal, so there is no
    # steady-state offset; CMA-ES only shapes how the arm gets there.
    w = theta[: ACTION * FEAT].reshape(ACTION, FEAT)
    b = np.zeros(ACTION, dtype=np.float32)
    return w, b


def policy(state, goal, w, b):
    # Residual linear policy: action = goal + W @ [q - goal, qd] + b, clipped
    # to the joint limits. The actuators are position servos, so the goal
    # itself is the baseline command; W and b learn corrections on top.
    # (W @ [q - goal, qd] alone can't output an absolute target.)
    feat = np.concatenate([state[:ACTION] - goal, state[ACTION:]]).astype(np.float32)
    return np.clip(goal + w @ feat + b, JOINT_LO, JOINT_HI).astype(np.float32)


@torch.no_grad()
def step_dynamics(model, state, action, x_mean, x_std, y_mean, y_std):
    # One world-model step: normalize [s, a], predict the normalized delta,
    # denormalize, and add it to the state.
    x = np.concatenate([state, action]).astype(np.float32)
    x_n = (x - x_mean) / x_std
    delta_n = model(torch.as_tensor(x_n).unsqueeze(0)).numpy()[0]
    return (state + (delta_n * y_std + y_mean)).astype(np.float32)


def sample_goal(rng):
    span = (JOINT_HI - JOINT_LO) * 0.5 * RANDOM_SCALE
    goal = HOME + rng.uniform(-span, span)
    return np.clip(goal, JOINT_LO, JOINT_HI).astype(np.float32)


def smooth_action(prev, raw):
    blended = (1.0 - ACTION_SMOOTH) * prev + ACTION_SMOOTH * raw
    delta = np.clip(blended - prev, -MAX_DELTA, MAX_DELTA)
    return np.clip(prev + delta, JOINT_LO, JOINT_HI).astype(np.float32)


def rollout(model, theta, x_mean, x_std, y_mean, y_std, rng):
    w, b = unpack(theta)
    state = np.concatenate([HOME, np.zeros(ACTION, dtype=np.float32)])
    goal = sample_goal(rng)
    prev_action = HOME.copy()
    total = 0.0
    for _ in range(HORIZON):
        raw = policy(state, goal, w, b)
        action = smooth_action(prev_action, raw)
        state = step_dynamics(model, state, action, x_mean, x_std, y_mean, y_std)
        err = state[:ACTION] - goal
        total -= float((err ** 2).sum())
        total -= VEL_PENALTY * float((state[ACTION:] ** 2).sum())
        total -= ACTION_SMOOTH_PENALTY * float(((action - prev_action) ** 2).sum())
        prev_action = action
    return total


def fitness(theta, model, x_mean, x_std, y_mean, y_std, seed):
    rng = np.random.default_rng(seed)
    returns = []
    for _ in range(EPISODES):
        returns.append(rollout(model, theta, x_mean, x_std, y_mean, y_std, rng))

    return -float(np.mean(returns))


def save_controller(theta):
    w, b = unpack(theta)
    payload = {
        "feature": "goal_plus_W_q_minus_goal_and_qd",
        "state_size": STATE,
        "action_size": ACTION,
        "W": w.astype(float).reshape(-1).tolist(),
        "b": b.astype(float).tolist(),
        "joint_lo": JOINT_LO.astype(float).tolist(),
        "joint_hi": JOINT_HI.astype(float).tolist(),
        "home": HOME.astype(float).tolist(),
        "vel_penalty": VEL_PENALTY,
        "horizon": HORIZON,
    }
    path = ROOT / "controller.json"
    path.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {path}")


def main():
    model = build_dynamics()
    x_mean, x_std, y_mean, y_std = load_norm()

    # Start CMA-ES from W = 0, b = 0: action = goal (send the goal straight
    # to the position servos), then learn corrections on top.
    theta0 = np.zeros(N_PARAMS, dtype=np.float32)

    print(f"initial imagined return: "
          f"{-fitness(theta0, model, x_mean, x_std, y_mean, y_std, seed=0):.2f}")
    es = cma.CMAEvolutionStrategy(
        theta0, 0.1, {"maxiter": CMA_ITERS, "seed": 0, "verb_disp": 1})
    es.optimize(lambda theta: fitness(theta, model, x_mean, x_std, y_mean, y_std, seed=0))
    print(f"best imagined return: {-es.result.fbest:.2f}")
    print(f"held-out imagined return (seed 1): "
          f"{-fitness(es.result.xbest, model, x_mean, x_std, y_mean, y_std, seed=1):.2f}")
    save_controller(es.result.xbest)


if __name__ == "__main__":
    current_time = time.time()
    main()
    end_time = time.time()
    print(f"Training time: {end_time - current_time} seconds")
