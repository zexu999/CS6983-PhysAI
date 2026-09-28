// AI USAGE: This file was written with assistance from Claude (Anthropic AI).
// Purpose: holds the level's script state (is the roadblock removed? is the level complete?)
// and decides whether a deposit counts. Also logs elapsed time on success and lets the
// player press R to restart.
using UnityEngine;
using UnityEngine.SceneManagement;

public class GameManager : MonoBehaviour
{
    public static GameManager Instance { get; private set; }

    public bool RoadblockRemoved { get; private set; }
    public bool LevelComplete { get; private set; }

    private float completionTime;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(this);
            return;
        }
        Instance = this;
    }

    private void OnDestroy()
    {
        if (Instance == this) Instance = null;
    }

    private void Update()
    {
        // Restart in case the payload gets stuck in a corner.
        if (Input.GetKeyDown(KeyCode.R))
        {
            SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
        }
    }

    /// <summary>Called by the Roadblock once it has been unlocked by the sensor.</summary>
    public void NotifyRoadblockRemoved()
    {
        RoadblockRemoved = true;
    }

    /// <summary>
    /// Called by the DepositZone when a Payload enters it.
    /// Success only counts if the roadblock has already been removed.
    /// </summary>
    public void TryCompleteDeposit(Payload payload)
    {
        if (LevelComplete) return;

        if (!RoadblockRemoved)
        {
            Debug.LogWarning("[GameManager] Payload entered the DepositZone before the roadblock was removed. This does NOT count.");
            return;
        }

        LevelComplete = true;
        // timeSinceLevelLoad starts at 0 when Play is pressed (and resets on R restart).
        completionTime = Time.timeSinceLevelLoad;
        Debug.Log($"[GameManager] SUCCESS! Payload '{payload.name}' deposited. Elapsed time from Play: {completionTime:F2} s");
    }

    private void OnGUI()
    {
        var style = new GUIStyle(GUI.skin.label) { fontSize = 20 };
        string status;
        if (LevelComplete)
            status = $"Deposited! Time: {completionTime:F2} s   (R to restart)";
        else if (RoadblockRemoved)
            status = "Door OPEN - push the crate into the green zone";
        else
            status = "Door LOCKED - find the yellow pressure plate";

        GUI.Label(new Rect(16, 12, 900, 30), status, style);
        GUI.Label(new Rect(16, 40, 900, 30), $"Time: {Time.timeSinceLevelLoad:F1} s   WASD: move   R: restart", style);
    }
}
