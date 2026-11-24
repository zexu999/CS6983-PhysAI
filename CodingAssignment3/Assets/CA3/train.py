import numpy as np
import torch
import time
from pathlib import Path
from torch import nn

ROOT = Path(__file__).resolve().parent / "Data"
STATE = 12  # UR5e: 6 joint angles (q) + 6 joint velocities (qd)
ACTION = 6  # UR5e: 6 position-servo targets (q*)


class DynamicsModel(nn.Module):
    """MLP world model: predicts the normalized next-state delta.

    Input : [q (6), qd (6), q* (6)] normalized  -> 18 dims
    Output: (s' - s) normalized                  -> 12 dims
    """

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


def pick_device():
    if torch.backends.mps.is_available():
        try:
            device = torch.device("mps")
            torch.zeros(1, device=device)
            return device
        except Exception:
            pass
    return torch.device("cpu")


def mean_std(z):
    return z.mean(0), z.std(0) + 1e-6


def norm(z, mean, std):
    return (z - mean) / std


def save_norm_json(x_mean, x_std, y_mean, y_std):
    arrays = {"x_mean": x_mean, "x_std": x_std, "y_mean": y_mean, "y_std": y_std}
    lines = ["{"]
    keys = list(arrays)
    for index, key in enumerate(keys):
        values = ", ".join(f"{float(value):.8f}" for value in arrays[key])
        comma = "," if index < len(keys) - 1 else ""
        lines.append(f'  "{key}": [{values}]{comma}')
    lines.append("}")
    (ROOT / "norm.json").write_text("\n".join(lines) + "\n")


def load_rollout():
    path = ROOT / "rollout.csv"
    if not path.exists() or path.stat().st_size == 0:
        raise FileNotFoundError(f"No rollout at {path}. Enable RolloutRecorder and Play.")
    data = np.loadtxt(path, delimiter=",", dtype=np.float32)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] != STATE + ACTION + STATE:
        raise ValueError(f"Expected {STATE + ACTION + STATE} columns, found {data.shape[1]}.")
    return data


def predict_delta(model, state, action, x_mean, x_std, y_mean, y_std):
    x = np.concatenate([state, action], axis=-1)
    x_n = (x - x_mean) / x_std
    with torch.no_grad():
        delta_n = model(torch.as_tensor(x_n)).numpy()
    return delta_n * y_std + y_mean


def joint_rmse(pred, true):
    err = pred[..., 0:ACTION] - true[..., 0:ACTION]
    return float(np.sqrt((err ** 2).sum(axis=-1).mean()))


def open_loop_rmse(model, data, start, x_mean, x_std, y_mean, y_std, horizon):
    end = len(data) - horizon
    if end <= start:
        return float("nan")
    idx = np.arange(start, end)
    if len(idx) > 2000:
        idx = idx[:: max(1, len(idx) // 2000)]
    state = data[idx, 0:STATE].copy()
    for t in range(horizon):
        action = data[idx + t, STATE:STATE + ACTION]
        state = state + predict_delta(model, state, action, x_mean, x_std, y_mean, y_std)
    return joint_rmse(state, data[idx + horizon, 0:STATE])


def multi_step_loss(model, data, split, x_mean, x_std, y_mean, y_std, horizon, batch, loss_fn):
    """Unroll the model `horizon` steps from random starts inside the train
    split and average the per-step state MSE against the recorded trajectory.
    Rows of `data` are consecutive FixedUpdate transitions [s, a, s'], so the
    true next state at unroll step t is row (start + t)'s s' column."""
    device = next(model.parameters()).device
    x_mean_t = torch.as_tensor(x_mean, dtype=torch.float32, device=device)
    x_std_t = torch.as_tensor(x_std, dtype=torch.float32, device=device)
    y_mean_t = torch.as_tensor(y_mean, dtype=torch.float32, device=device)
    y_std_t = torch.as_tensor(y_std, dtype=torch.float32, device=device)

    idx = np.random.randint(0, split - horizon, size=batch)
    state = torch.as_tensor(data[idx, 0:STATE], device=device)
    loss = 0.0
    for t in range(horizon):
        action = torch.as_tensor(data[idx + t, STATE:STATE + ACTION], device=device)
        x = torch.cat([state, action], dim=-1)
        delta = model((x - x_mean_t) / x_std_t) * y_std_t + y_mean_t
        state = state + delta
        true_next = torch.as_tensor(data[idx + t, STATE + ACTION:], device=device)
        loss = loss + loss_fn(state, true_next)
    return loss / horizon


def train_world_model(data):
    device = pick_device()
    print(f"Training on {device} with {len(data)} transitions.")

    split = int(0.9 * len(data))
    train, val = data[:split], data[split:]

    # Normalization stats over inputs [s, a] and delta targets (s' - s).
    x = np.concatenate([train[:, 0:STATE], train[:, STATE:STATE + ACTION]], axis=-1)
    y = train[:, STATE + ACTION:] - train[:, 0:STATE]
    x_mean, x_std = mean_std(x)
    y_mean, y_std = mean_std(y)

    model = DynamicsModel().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-5)
    epochs = 100
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
    loss_fn = nn.MSELoss()

    x_mean_t = torch.as_tensor(x_mean, device=device)
    x_std_t = torch.as_tensor(x_std, device=device)
    y_mean_t = torch.as_tensor(y_mean, device=device)
    y_std_t = torch.as_tensor(y_std, device=device)
    x_t = torch.as_tensor(x, device=device)
    y_t = torch.as_tensor(y, device=device)
    batch = 256
    horizon = 5

    for epoch in range(epochs):
        model.train()
        perm = torch.randperm(len(train), device=device)
        epoch_loss = 0.0
        n_batches = 0
        for i in range(0, len(train), batch):
            idx = perm[i:i + batch]
            x_n = (x_t[idx] - x_mean_t) / x_std_t
            y_n = (y_t[idx] - y_mean_t) / y_std_t
            loss = loss_fn(model(x_n), y_n)
            # Multi-step unroll loss keeps long-horizon imagination honest.
            loss = loss + multi_step_loss(
                model, train, len(train), x_mean, x_std, y_mean, y_std,
                horizon, batch=64, loss_fn=loss_fn)
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            epoch_loss += float(loss.detach())
            n_batches += 1
        scheduler.step()
        if (epoch + 1) % 10 == 0:
            print(f"epoch {epoch + 1:3d}  loss {epoch_loss / n_batches:.6f}")

    # Evaluate on CPU (the helpers below build CPU tensors).
    model.eval().to("cpu")
    val_pred = val[:, 0:STATE] + predict_delta(
        model, val[:, 0:STATE], val[:, STATE:STATE + ACTION],
        x_mean, x_std, y_mean, y_std)
    one_step = joint_rmse(val_pred, val[:, STATE + ACTION:])
    ol10 = open_loop_rmse(model, val, 0, x_mean, x_std, y_mean, y_std, 10)
    ol50 = open_loop_rmse(model, val, 0, x_mean, x_std, y_mean, y_std, 50)
    ol100 = open_loop_rmse(model, val, 0, x_mean, x_std, y_mean, y_std, 100)
    print(f"val one-step joint RMSE: {one_step:.6f} rad")
    print(f"open-loop RMSE 10/50/100 steps: {ol10:.6f} / {ol50:.6f} / {ol100:.6f} rad")

    ROOT.mkdir(parents=True, exist_ok=True)
    torch.save(model.state_dict(), ROOT / "dynamics.pt")
    torch.onnx.export(
        model, torch.randn(1, STATE + ACTION), ROOT / "dynamics.onnx",
        input_names=["input"], output_names=["output"],
        opset_version=13, dynamo=False)
    save_norm_json(x_mean, x_std, y_mean, y_std)
    np.savez(ROOT / "norm.npz",
             x_mean=x_mean, x_std=x_std, y_mean=y_mean, y_std=y_std)
    print(f"saved dynamics.pt, dynamics.onnx, norm.json, norm.npz to {ROOT}")


if __name__ == "__main__":
    current_time = time.time()
    train_world_model(load_rollout())
    end_time = time.time()
    print(f"Training time: {end_time - current_time} seconds")
