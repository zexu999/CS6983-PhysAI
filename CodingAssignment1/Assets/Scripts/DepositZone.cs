// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: trigger volume at the end of the maze. Only a Payload counts (the Actor
// walking in does nothing). The GameManager decides whether the deposit is valid.
using UnityEngine;

[RequireComponent(typeof(Collider))]
public class DepositZone : MonoBehaviour
{
    private void Reset()
    {
        GetComponent<Collider>().isTrigger = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        Rigidbody rb = other.attachedRigidbody;
        if (rb == null) return;

        Payload payload = rb.GetComponent<Payload>();
        if (payload == null) return;

        if (GameManager.Instance != null) GameManager.Instance.TryCompleteDeposit(payload);
    }
}
