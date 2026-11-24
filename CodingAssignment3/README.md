# Coding Assignment #3: UR5e World Model + IK

UR5e arm from MuJoCo Menagerie running in Unity with the MuJoCo plugin. I collect random joint-goal rollouts, train an MLP world model, train a linear controller with CMA-ES inside that model, and use DLS IK so the arm follows a target sphere you can drag around.

- **Report (setup, numbers, design question):** [REPORT.md](REPORT.md)
- **Step 5 reflection:** [REFLECTION.md](REFLECTION.md)
- **Demo video:** _link here_

## Results at a glance
| | |
|---|---|
| World model one-step joint RMSE | 0.045 rad |
| Open-loop RMSE 10 / 50 / 100 steps | 0.23 / 0.47 / 0.64 rad |
| CMA-ES imagined return | −422 → −342 |
| Live joint tracking error | ≈ 0.01 rad |
| Tool point to target at rest | ≈ 1 cm |

## Files
- `Assets/CA3/`: all scripts
  - `ArmAgent.cs`, `ArmJointController.cs`, `RolloutRecorder.cs`, `WorldModelController.cs`, `WorldModelDriver.cs`, `IkGoalProvider.cs`
  - `train.py` (world model), `train_controller.py` (CMA-ES controller)
  - `Data/`: `rollout.csv`, `dynamics.onnx`, `dynamics.pt`, `norm.json`, `controller.json`
- `Assets/CA3.unity`: the scene, with the scripts on the `base` body
- `Assets/base.prefab`: ghost arm prefab

## How to run
1. Unity 6000.3.24f1. Clone MuJoCo and check out `3.17.0`. Point `org.mujoco` in `Packages/manifest.json` at your `mujoco/unity` folder (it currently uses my local path). `Assets/mujoco.dylib` is the macOS 3.17.0 library.
2. Open `Assets/CA3.unity`. The components are on `base`.
3. **Collect data:** enable ArmAgent, ArmJointController (Emit Actions on) and RolloutRecorder, then Play. Time Scale 5 takes about 5 minutes.
4. **Train:** `cd Assets/CA3 && pip install torch numpy cma onnx && python train.py && python train_controller.py`
5. **Verify with random goals:** enable WorldModelController and WorldModelDriver, then tick Emit Actions off and Verify Random Goals on.
6. **Track the target:** Verify Random Goals off, enable IkGoalProvider, Play, and drag `TrackingTarget` in the Scene view.

## Changes from the skeleton worth knowing
- ArmAgent searches the whole scene for actuators, because the importer puts them next to `base`, not under it.
- The ghost arm is posed per body from MuJoCo FK, and CA3 scripts are stripped from the ghost copy.
- The policy is `goal + W·[q − goal, qd]` with no bias (see REPORT.md for why).
- IK aims at `attachment_site` and stays inside the joint range of the training data.
- ArmJointController has a `verifyRandomGoals` switch for Step 3.3.
