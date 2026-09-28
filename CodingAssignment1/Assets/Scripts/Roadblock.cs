// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: a door that physically blocks the corridor to the DepositZone.
// When unlocked, its collider is disabled immediately (so it stops blocking), it slides
// into the floor, logs that it was removed, and tells the GameManager.
using System.Collections;
using UnityEngine;

public class Roadblock : MonoBehaviour
{
    public float sinkDistance = 3.2f;
    public float sinkDuration = 1.0f;

    private bool removed;

    public void Unlock()
    {
        if (removed) return;
        removed = true;

        foreach (var c in GetComponentsInChildren<Collider>()) c.enabled = false;

        Debug.Log("[Roadblock] Roadblock has been removed - the door is open!");
        if (GameManager.Instance != null) GameManager.Instance.NotifyRoadblockRemoved();

        StartCoroutine(Sink());
    }

    private IEnumerator Sink()
    {
        Vector3 start = transform.position;
        Vector3 end = start + Vector3.down * sinkDistance;
        float t = 0f;
        while (t < sinkDuration)
        {
            t += Time.deltaTime;
            transform.position = Vector3.Lerp(start, end, t / sinkDuration);
            yield return null;
        }
        gameObject.SetActive(false);
    }
}
