// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: angled top-down camera that smoothly follows the Actor.
// Runs in LateUpdate so it moves after the actor has moved this frame.
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3 offset = new Vector3(0f, 18f, -11f);
    public float smoothTime = 0.15f;

    private Vector3 velocity;

    private void LateUpdate()
    {
        if (target == null) return;
        Vector3 desired = target.position + offset;
        transform.position = Vector3.SmoothDamp(transform.position, desired, ref velocity, smoothTime);
        transform.LookAt(target.position);
    }
}
