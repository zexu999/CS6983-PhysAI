# Session 2 Participation — In-Class Assignment

CS6983 Phys AI, World Model and Games — Week 2 in-class activity on Unity physics (Rigidbody forces, triggers, events).

## Scene

`Session2.unity` contains:

| GameObject | Setup |
|---|---|
| Floor | Large plane / scaled cube with a collider |
| Actor | Cube with `Rigidbody` + `InClassActorController.cs` (Target dragged into Inspector) |
| Target | Empty or small marker object placed away from the Actor |
| Payload | Sphere with `Rigidbody`, **Tag = "Payload"** |
| Goal Zone | Cube with collider **Is Trigger = true** + `InClassGoalZone.cs` |
| Camera | Positioned to view the play area |
| Light | Directional light |

At runtime the Actor applies a constant force toward the Target on the ground plane. When the Payload enters the Goal Zone trigger, the run is marked successful and the `PayloadSucceeded` event fires with the time-to-success in seconds (logged to the Console).

## Scripts

- `InClassActorController.cs` — TODO: apply force toward the target → `_body.AddForce(toTarget.normalized * moveStrength)` in `FixedUpdate()`.
- `InClassGoalZone.cs` — TODOs: `IsPayload()` returns `other.CompareTag(payloadTag)`; `OnTriggerEnter` sets `_succeeded = true`, records `TimeToSuccess = Time.time - _runStartTime`, and invokes `PayloadSucceeded?.Invoke(TimeToSuccess)`.


## Experiment Observations

Baseline settings — Actor: Mass 1, Linear Damping 0, Angular Damping 0.05. Payload (sphere): Mass 1, Linear Damping 1, Angular Damping 5. `moveStrength = 20`.
Only one parameter was changed per run; everything else stayed at baseline. Time to success is from the `PayloadSucceeded` event / Console log.

| # | Object | Parameter changed | Baseline → New | Time to success | Observation |
|---|---|---|---|---|---|
| 0 | – | Baseline | – | **1.26 s** | Actor pushes the ball straight into the Goal Zone. |
| 1 | Payload | Mass | 1 → 5 | **3.90 s** | The heavier ball accelerates much more slowly when hit (F = ma), so the Actor has to keep pushing it. |
| 2 | Payload | Mass | 1 → 10 | **Did not reach goal** | The ball is too heavy for the Actor's fixed force (plus the ball's linear damping), so it never reaches the Goal Zone. |
| 3 | Actor | Linear Damping | 0 → 5 | **2.32 s** | Damping acts like drag: the Actor accelerates slower and hits the ball with less momentum, nearly doubling the time. |
| 4 | Payload | Angular Damping | 5 → 0.05 | **1.22 s** | The ball rolls more freely after being hit, so it arrives slightly faster, but the difference is small because the Actor pushes it the whole way. |

**Takeaway:** Payload mass had the biggest effect (it can make the task fail entirely), Actor linear damping had a moderate effect, and Payload angular damping had only a minor
effect.
### Notes
- Linear Damping = drag on translational motion; Angular Damping = drag on rotation. Both are in the Rigidbody component in the Inspector.
- Mass only matters in combination with forces (F = ma) — with no force applied, changing mass alone shows little difference.
- Tip: to compare runs fairly, reset the scene (stop/play) between runs so `ResetRun()` re-arms the timer via `OnEnable()`.
