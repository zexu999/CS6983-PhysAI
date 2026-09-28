// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: a pressure-plate sensor. When the Actor steps on it, it unlocks the linked Roadblock.
// Only the Actor can trigger it (the payload cannot), and it only fires once.
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class SensorTrigger : MonoBehaviour
{
    public Roadblock target;
    public Renderer plateRenderer;
    public Color activatedColor = new Color(0.2f, 0.9f, 0.2f);

    private bool triggered;

    private void Reset()
    {
        GetComponent<Collider>().isTrigger = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (triggered) return;

        // Use the attached Rigidbody so child colliders of the actor also count.
        Rigidbody rb = other.attachedRigidbody;
        if (rb == null || rb.GetComponent<ActorController>() == null) return;

        triggered = true;
        Debug.Log("[SensorTrigger] Actor stepped on the pressure plate.");
        if (plateRenderer != null) plateRenderer.material.color = activatedColor;
        if (target != null) target.Unlock();
    }
}
