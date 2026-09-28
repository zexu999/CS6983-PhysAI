# Coding Assignment #1: Physics Maze Chamber

Use WASD to move a blue Actor through a maze. First step on the yellow pressure plate to open the red door. Then push the brown crate (the Payload) through the door and into the green DepositZone.

## How to run

1. In Unity Hub, click **Add** and select the `CodingAssignment1` folder (Unity 6.3 LTS).
2. Open `Assets/Scenes/SampleScene`.
3. Press **Play**.

Note: *Active Input Handling* is set to **Both** in Player Settings (the scripts use the legacy Input Manager).

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

For a run to count as a success, the Actor must first physically step on the yellow pressure plate to make the red door sink. In the script, this sets a boolean state to true and logs "Roadblock has been removed". Finally, the player must push the wooden box into the bottom-left green area, triggering the "SUCCESS!" log. A player might try to cheat by pushing the box into the green area before triggering the sensor. My design blocks this physically with the red door blocking the path, and in the script by ensuring the deposit zone only accepts the payload if the roadblock state is already cleared.

### 2. One unexpected behavior while building: cause and fix

While building the scene, I encountered an unexpected behavior where the Actor was completely unable to push the wooden box. This was caused by the Actor having insufficient force to overcome the box's high friction. To fix this, I increased the Actor Move Force from 25 to 40. I also decreased the Payload Friction from 0.6 to 0.3, which immediately allowed the Actor to smoothly push the payload through the maze.

### 3. One design decision: what I optimized for and what I gave up

I made a specific design decision to include an input mapping where pressing the 'R' key restarts the level. I optimized for player fairness and control feel, knowing that physics objects like the box can easily get stuck in tight corners. By adding this reset feature, I gave up the strict penalty of forcing the player to flawlessly navigate the maze on their first try, ensuring the game remains enjoyable rather than frustrating.

### 4. Weakest part of the system and what I would change first

The weakest part of my system right now is the robustness of the collision geometry in tight spaces, as the box can still occasionally get stuck against the sharp 90-degree wall corners (which necessitated the 'R' restart feature). If I could change one thing first, I would replace the Payload's Box Collider with a Sphere Collider, or add rounded collision helper-meshes to the maze corners. This would prevent the payload from physically snagging on the vertices when pushed at an angle, making the pushing mechanics much smoother without needing manual level resets.
