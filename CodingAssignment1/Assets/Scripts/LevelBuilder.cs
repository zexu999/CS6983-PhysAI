// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: builds the whole physics chamber (maze, actor, payload, sensor, roadblock,
// DepositZone, physics materials, camera) from a text layout when Play is pressed.
// All physics values are exposed in the Inspector so they can be tuned.
//
// Layout legend:
//   #  wall          .  floor
//   A  actor start   P  payload (crate)
//   S  sensor plate  D  roadblock door
//   Z  DepositZone
using UnityEngine;

public class LevelBuilder : MonoBehaviour
{
    [Header("Layout (row 0 = far/top side of the screen)")]
    public string[] layout =
    {
        "#################",
        "#A..P.....#.....#",
        "#.#######.#.###.#",
        "#.......#.#...#S#",
        "#########.###.###",
        "#Z......D.......#",
        "#################",
    };

    [Tooltip("Width of one grid cell in meters. Wide corridors leave room to walk around the crate at corners.")]
    public float cellSize = 4f;
    public float wallHeight = 3f;

    [Header("Actor physics (light, low friction, some drag)")]
    public float actorMass = 1f;
    public float actorDrag = 1.5f;
    public float actorFriction = 0.1f;
    public float actorMoveForce = 25f;
    public float actorMaxSpeed = 6f;

    [Header("Payload physics (heavy, high friction, low drag)")]
    public float payloadMass = 2.5f;
    public float payloadDrag = 0.5f;
    public float payloadFriction = 0.6f;
    public float payloadSize = 1.4f;

    [Header("Floor / wall physics")]
    public float floorFriction = 0.4f;
    public float wallFriction = 0.0f; // frictionless walls so nothing sticks when sliding along them

    [Header("Colors")]
    public Color floorColor = new Color(0.75f, 0.75f, 0.72f);
    public Color wallColor = new Color(0.35f, 0.38f, 0.45f);
    public Color actorColor = new Color(0.2f, 0.5f, 1f);
    public Color payloadColor = new Color(0.65f, 0.42f, 0.2f);
    public Color sensorColor = new Color(1f, 0.85f, 0.1f);
    public Color doorColor = new Color(0.85f, 0.15f, 0.15f);
    public Color zoneColor = new Color(0.2f, 0.85f, 0.35f);

    private Transform root;

    private void Awake()
    {
        if (GetComponent<GameManager>() == null) gameObject.AddComponent<GameManager>();
        root = new GameObject("GeneratedLevel").transform;
        Build();
    }

    private Vector3 CellToWorld(int col, int row) => new Vector3(col * cellSize, 0f, -row * cellSize);

    private void Build()
    {
        int rows = layout.Length;
        int cols = 0;
        foreach (var line in layout) cols = Mathf.Max(cols, line.Length);

        BuildFloor(rows, cols);

        Roadblock door = null;
        SensorTrigger sensor = null;
        Transform actor = null;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < layout[r].Length; c++)
            {
                Vector3 p = CellToWorld(c, r);
                switch (layout[r][c])
                {
                    case '#': BuildWall(p); break;
                    case 'A': actor = BuildActor(p); break;
                    case 'P': BuildPayload(p); break;
                    case 'S': sensor = BuildSensor(p); break;
                    case 'D': door = BuildDoor(p); break;
                    case 'Z': BuildDepositZone(p); break;
                }
            }
        }

        if (sensor != null) sensor.target = door;
        else Debug.LogError("[LevelBuilder] Layout has no sensor 'S'.");
        if (door == null) Debug.LogError("[LevelBuilder] Layout has no roadblock 'D'.");

        SetupCamera(actor);
    }

    // ---------- builders ----------

    private GameObject MakeCube(string name, Vector3 pos, Vector3 scale, Color color)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Cube);
        go.name = name;
        go.transform.SetParent(root);
        go.transform.position = pos;
        go.transform.localScale = scale;
        go.GetComponent<Renderer>().material.color = color;
        return go;
    }

    private static void RemoveCollider(GameObject go)
    {
        var col = go.GetComponent<Collider>();
        col.enabled = false;
        Destroy(col);
    }

    private void BuildFloor(int rows, int cols)
    {
        float w = cols * cellSize, d = rows * cellSize;
        Vector3 center = new Vector3((cols - 1) * cellSize * 0.5f, -0.25f, -(rows - 1) * cellSize * 0.5f);
        var floor = MakeCube("Floor", center, new Vector3(w, 0.5f, d), floorColor);
        PhysicsCompat.ApplyMaterial(floor.GetComponent<Collider>(), "FloorMat", floorFriction, floorFriction, 0f);
    }

    private void BuildWall(Vector3 p)
    {
        var wall = MakeCube("Wall", p + Vector3.up * wallHeight * 0.5f,
                            new Vector3(cellSize, wallHeight, cellSize), wallColor);
        PhysicsCompat.ApplyMaterial(wall.GetComponent<Collider>(), "WallMat", wallFriction, wallFriction, 0f);
    }

    private Transform BuildActor(Vector3 p)
    {
        var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        go.name = "Actor";
        go.transform.SetParent(root);
        go.transform.position = p + Vector3.up * 0.5f;
        go.GetComponent<Renderer>().material.color = actorColor;
        PhysicsCompat.ApplyMaterial(go.GetComponent<Collider>(), "ActorMat", actorFriction, actorFriction, 0f);

        var rb = go.AddComponent<Rigidbody>();
        rb.mass = actorMass;
        PhysicsCompat.SetDamping(rb, actorDrag, 0.05f);
        rb.freezeRotation = true; // slide instead of roll, so control feels predictable
        rb.interpolation = RigidbodyInterpolation.Interpolate;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;

        var ctrl = go.AddComponent<ActorController>();
        ctrl.moveForce = actorMoveForce;
        ctrl.maxSpeed = actorMaxSpeed;
        return go.transform;
    }

    private void BuildPayload(Vector3 p)
    {
        var go = MakeCube("Payload_Crate", p + Vector3.up * payloadSize * 0.5f,
                          Vector3.one * payloadSize, payloadColor);
        PhysicsCompat.ApplyMaterial(go.GetComponent<Collider>(), "PayloadMat", payloadFriction, payloadFriction, 0f);

        var rb = go.AddComponent<Rigidbody>();
        rb.mass = payloadMass;
        PhysicsCompat.SetDamping(rb, payloadDrag, 0.5f);
        // Allow spinning around Y, but never tipping over.
        rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
        rb.interpolation = RigidbodyInterpolation.Interpolate;

        go.AddComponent<Payload>();
    }

    private SensorTrigger BuildSensor(Vector3 p)
    {
        // Trigger volume (tall enough that the actor always overlaps it).
        var trigger = new GameObject("Sensor_PressurePlate");
        trigger.transform.SetParent(root);
        trigger.transform.position = p;
        var box = trigger.AddComponent<BoxCollider>();
        box.isTrigger = true;
        box.size = new Vector3(cellSize * 0.7f, 2f, cellSize * 0.7f);
        box.center = new Vector3(0f, 1f, 0f);

        // Visual plate (no collider).
        var plate = MakeCube("PlateVisual", p + Vector3.up * 0.03f,
                             new Vector3(cellSize * 0.7f, 0.06f, cellSize * 0.7f), sensorColor);
        RemoveCollider(plate);
        plate.transform.SetParent(trigger.transform, true);

        var sensor = trigger.AddComponent<SensorTrigger>();
        sensor.plateRenderer = plate.GetComponent<Renderer>();
        return sensor;
    }

    private Roadblock BuildDoor(Vector3 p)
    {
        var go = MakeCube("Roadblock_Door", p + Vector3.up * wallHeight * 0.5f,
                          new Vector3(cellSize, wallHeight, cellSize), doorColor);
        PhysicsCompat.ApplyMaterial(go.GetComponent<Collider>(), "DoorMat", wallFriction, wallFriction, 0f);
        var door = go.AddComponent<Roadblock>();
        door.sinkDistance = wallHeight + 0.2f;
        return door;
    }

    private void BuildDepositZone(Vector3 p)
    {
        var zone = new GameObject("DepositZone");
        zone.transform.SetParent(root);
        zone.transform.position = p;
        var box = zone.AddComponent<BoxCollider>();
        box.isTrigger = true;
        box.size = new Vector3(cellSize * 0.8f, 2f, cellSize * 0.8f);
        box.center = new Vector3(0f, 1f, 0f);
        zone.AddComponent<DepositZone>();

        var visual = MakeCube("ZoneVisual", p + Vector3.up * 0.03f,
                              new Vector3(cellSize * 0.8f, 0.06f, cellSize * 0.8f), zoneColor);
        RemoveCollider(visual);
        visual.transform.SetParent(zone.transform, true);
    }

    private void SetupCamera(Transform actor)
    {
        Camera cam = Camera.main;
        if (cam == null)
        {
            var camGo = new GameObject("Main Camera");
            camGo.tag = "MainCamera";
            cam = camGo.AddComponent<Camera>();
        }
        var follow = cam.GetComponent<CameraFollow>();
        if (follow == null) follow = cam.gameObject.AddComponent<CameraFollow>();
        follow.target = actor;
        if (actor != null) cam.transform.position = actor.position + follow.offset;
    }
}
