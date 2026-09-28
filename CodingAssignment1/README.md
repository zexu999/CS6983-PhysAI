# Coding Assignment #1: Physics Maze Chamber

Use WASD to move a blue Actor through a maze. First step on the yellow pressure plate to open the red door. Then push the brown crate (the Payload) through the door and into the green DepositZone.

## How to run

1. Create a new **3D** Unity project (Unity 2022 LTS or Unity 6; Built-in or URP both work).
2. Copy the `Assets/Scripts` folder into the project's `Assets` folder.
3. In the default `SampleScene`, create an empty GameObject named `Level` and add the **LevelBuilder** component to it. (LevelBuilder adds the GameManager itself.)
4. Keep the default Main Camera and Directional Light. The builder attaches `CameraFollow` to the camera.
5. Press **Play**.

When you press Play, the level is generated from the text grid in `LevelBuilder.layout`. You can edit the grid in the Inspector.

| Key | Action |
|---|---|
| W A S D | Move the Actor |
| R | Restart the level (e.g. if the crate gets stuck in a corner) |

## Scripts

| Script | Responsibility |
|---|---|
| `LevelBuilder` | Builds the maze, Actor, Payload, sensor, door, DepositZone, and physics materials from the layout grid. It also exposes all the physics tuning values. |
| `ActorController` | Reads WASD in `Update()` and applies `AddForce` in `FixedUpdate()`. Caps the Actor's horizontal speed. |
| `SensorTrigger` | Pressure plate. `OnTriggerEnter` fires only for the Actor and only once, then calls `Roadblock.Unlock()`. |
| `Roadblock` | Door that blocks the only path to the DepositZone. On unlock it disables its collider, logs `Roadblock has been removed`, notifies the GameManager, and sinks into the floor. |
| `Payload` | Marker component on the crate. It moves only through physics collisions. |
| `DepositZone` | Trigger. Only a Payload counts, and it asks the GameManager whether the deposit is valid. |
| `GameManager` | Level state (`RoadblockRemoved`, `LevelComplete`). Rejects early deposits, logs elapsed time on success, and shows the HUD. |
| `CameraFollow` | Smooth top-down follow camera that runs in `LateUpdate()`. |
| `PhysicsCompat` | Handles Unity 2022 vs Unity 6 API naming (`drag`/`linearDamping`, `PhysicMaterial`/`PhysicsMaterial`). |

## Physics tuning (intentionally different values)

| Object | Mass | Linear drag | Friction |
|---|---|---|---|
| Actor (sphere, rotation frozen) | 1.0 | 1.5 | 0.1 |
| Payload (crate, tip-over locked) | 2.5 | 0.5 | 0.3 |
| Floor | – | – | 0.4 |
| Walls / door | – | – | 0.0 |

Actor move force is 40 N and max speed is 6 m/s. All values can be changed in the LevelBuilder Inspector.

## Expected Console output

```
[SensorTrigger] Actor stepped on the pressure plate.
[Roadblock] Roadblock has been removed - the door is open!
[GameManager] SUCCESS! Payload 'Payload_Crate' deposited. Elapsed time from Play: 48.73 s
```
If the Payload enters the DepositZone while the door is still locked, the GameManager logs a warning and the level does not complete.

## AI usage

All C# scripts in `Assets/Scripts` were written with help from Claude (Anthropic AI). Each file says so in a header comment. **The reflection answers below were written by me without AI.**

---

## Reflection

### 1. What must be true for a run to count as success? How could a player cheat, and does the design block it?

_(your answer)_

### 2. One unexpected behavior while building: cause and fix

_(your answer)_

### 3. One design decision: what I optimized for and what I gave up

_(your answer)_

### 4. Weakest part of the system and what I would change first

_(your answer)_
