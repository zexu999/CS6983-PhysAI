# CA3 Report

**Name:** Zexu Wang
**Unity version:** 6.3 LTS (6000.3.24f1)
**MuJoCo plugin version:** 3.17.0
**Chosen arm (Menagerie):** Universal Robots UR5e (6 DOF)
**Model link:** https://github.com/google-deepmind/mujoco_menagerie/tree/main/universal_robots_ur5e

## Why this arm
The UR5e has 6 hinge joints and no gripper, so the state is 12 numbers (q, qd) and the action is 6 joint targets. That kept the world model and the CMA-ES search small, and it is not the Franka Panda from the lecture demo.

## Pipeline summary
1. **Data**: ArmJointController picks a random joint goal every 3 s (home ± 35% of each joint's half range) and sends it to the position actuators. RolloutRecorder logs one row per physics step: q, qd, q*, q', qd' (30 columns). 70,000 steps → 69,999 transitions in `Data/rollout.csv`.
2. **World model** (`train.py`): MLP, 18 → 256 → 256 → 256 → 12, predicts the normalized change in state. Loss = one-step MSE + a 5-step unroll MSE. Exported to `dynamics.onnx` + `norm.json` for Unity.
3. **Controller** (`train_controller.py`): linear policy trained with CMA-ES only inside the world model (200-step imagined rollouts, 16 random goals per evaluation).
4. **Unity**: IkGoalProvider turns the target sphere position into a joint goal, WorldModelController turns (state, goal) into an action, and WorldModelDriver runs the ONNX model live to draw a ghost arm and a 100-step forecast line.

## World model results
- DOF (`ACTION`): 6
- One-step joint RMSE (validation): **0.045 rad**
- Open-loop 10 / 50 / 100-step RMSE: **0.234 / 0.471 / 0.638 rad**
- Training: 100 epochs on Apple MPS, about 7 minutes.

One step is accurate, but the error grows when the model is rolled out on its own predictions. This is why the ghost arm is softly pulled back toward the real arm and re-anchored.

## Controller results
- Policy: `action = goal + W · [q − goal, qd]`, clipped to joint limits (W is 6×12, no bias).
- Start point: W = 0, which means "send the goal straight to the position servos".
- Imagined return (CMA-ES, 60 iterations): **−422 → −342** (held-out seed: −370).
- Final joint error after 300 imagined steps (50 random goals): mean **0.128 rad**, worst joint **0.317 rad** (W = 0 gives 0.125 / 0.367).
- Live in Unity (random-goal verify mode and IK mode): max joint tracking error settles to about **0.01 rad**.

The first version from the skeleton used `action = W · [q − goal, qd] + b`. Because the actuators take absolute joint targets, this policy cannot output "go to the goal" (it never sees the goal itself), so the arm stopped far from the target (imagined return −494, about 0.65 rad error per joint). Adding the goal as the baseline fixed it. Removing the bias then removed a constant offset of about 0.24 rad, because with qd = 0 the only resting point is q = goal.

## WorldModelDriver
- **Predict**: build [q, qd, q*] → normalize with norm.json → ONNX (InferenceEngine, CPU) → denormalize the delta → add to the ghost state.
- **Reanchor**: copy the real state into the ghost when the joint error goes over 0.2 rad, or every 3000 steps. In between, the ghost is blended 5% per step toward the real state.
- **Forecast**: from the ghost state, roll the world model + learned policy 100 steps ahead and draw the end-effector path (yellow line). A second short line connects the ghost and real end effectors.
- The ghost arm is a copy of the arm prefab with all MuJoCo and CA3 scripts removed. Each ghost body is posed from MuJoCo forward kinematics (xpos / xquat), the same way the plugin poses the real bodies.

## IK + live target
- **Method**: damped least squares (λ = 0.05) on the 3D position of `attachment_site`, with a numerical 3×6 Jacobian. Forward kinematics comes from MuJoCo itself: write candidate q into mjData, run mj_kinematics, read the tool point, then restore the real qpos. Up to 50 iterations per physics step, warm-started from the last solution, with 4 random restarts if the error stays above 5 mm.
- **Data-range limit**: the solution is clamped to the same joint box the rollout data came from (home ± 35% of half range). Without this, a far target made IK jump to a solution more than 3 rad away from anything in the data, and the controller got stuck there (joint error 3.7 rad, end effector about 1 m from the target).
- **Result**: IK residual under 0.5 cm. At rest the tool point sits **about 1 cm** from the sphere.
- **Failure cases**: targets outside the data box or outside reach, where the arm goes to the closest point it can reach and stops. Only position is solved, not orientation.

## Design question
Why can rollout collection stay in joint space while deployment tracks a Cartesian target?

The world model and the controller only care about how the joints move: given q, qd and a joint target, what happens next. That is a property of the arm, so joint space is enough to learn it. A Cartesian target only matters at the very end, and IK handles it by turning the target into a joint goal using pure kinematics. The condition is that IK has to give goals that look like the ones in the training data. In my run this was not automatic: I had to clamp IK to the data range before tracking worked.
