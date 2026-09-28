// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: WASD movement for the Actor using physics forces.
// Input is read in Update (every frame, so no key press is missed) and the force is
// applied in FixedUpdate (the physics step), which is the correct place for AddForce.
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class ActorController : MonoBehaviour
{
    [Tooltip("Force applied per physics step while a key is held (Newtons).")]
    public float moveForce = 25f;

    [Tooltip("Horizontal speed cap (m/s) so the actor does not accelerate forever.")]
    public float maxSpeed = 6f;

    private Rigidbody rb;
    private Vector3 inputDir;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        // Read WASD explicitly.
        float x = 0f, z = 0f;
        if (Input.GetKey(KeyCode.W)) z += 1f;
        if (Input.GetKey(KeyCode.S)) z -= 1f;
        if (Input.GetKey(KeyCode.D)) x += 1f;
        if (Input.GetKey(KeyCode.A)) x -= 1f;

        inputDir = new Vector3(x, 0f, z);
        if (inputDir.sqrMagnitude > 1f) inputDir.Normalize(); // no faster diagonals
    }

    private void FixedUpdate()
    {
        rb.AddForce(inputDir * moveForce, ForceMode.Force);

        // Clamp horizontal speed, keep vertical (gravity) untouched.
        Vector3 v = PhysicsCompat.GetVelocity(rb);
        Vector3 flat = new Vector3(v.x, 0f, v.z);
        if (flat.magnitude > maxSpeed)
        {
            flat = flat.normalized * maxSpeed;
            PhysicsCompat.SetVelocity(rb, new Vector3(flat.x, v.y, flat.z));
        }
    }
}
