using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

// Runs after MjScene (order 0): the state observed here is the post-step
// state, so each recorded (s, a, s') row is correctly aligned — the action
// in the row is the one MuJoCo actually applied to produce s'.
[DefaultExecutionOrder(10)]
public class RolloutRecorder : MonoBehaviour
{
    public ArmAgent agent;
    public ArmJointController controller;
    public int steps = 70000;

    // Where the CSV goes, relative to the Assets folder. train.py looks for
    // rollout.csv in the Data folder next to itself, so keep this in sync
    // with where you put the scripts (default: Assets/CA3/Data).
    public string outputSubdir = "CA3/Data";

    readonly List<float[]> states = new();
    readonly List<float[]> actions = new();
    readonly List<float[]> nextStates = new();

    int n;
    float[] prevS;
    float[] prevA;
    bool saved;

    void Start()
    {
        if (agent == null)
            agent = GetComponent<ArmAgent>();
        if (controller == null)
            controller = GetComponent<ArmJointController>();
        if (agent == null || !agent.Ready)
        {
            Debug.LogError("RolloutRecorder needs a ready ArmAgent.", this);
            enabled = false;
            return;
        }

        prevS = new float[agent.StateSize];
        prevA = new float[agent.ActionSize];
        states.Clear();
        actions.Clear();
        nextStates.Clear();
        n = 0;
        saved = false;
        Debug.Log($"RolloutRecorder: collecting {steps} steps " +
                  $"({agent.StateSize + agent.ActionSize + agent.StateSize} columns per row).", this);
    }

    void FixedUpdate()
    {
        if (saved || agent == null || !agent.Ready)
            return;

        float[] state = agent.Observe();
        float[] action = agent.CurrentAction;

        if (n > 0)
        {
            // The state observed now is the result of the previous action.
            states.Add((float[])prevS.Clone());
            actions.Add((float[])prevA.Clone());
            nextStates.Add((float[])state.Clone());
        }

        Array.Copy(state, prevS, prevS.Length);
        Array.Copy(action, prevA, prevA.Length);
        n++;

        if (n % 5000 == 0)
            Debug.Log($"RolloutRecorder: {n}/{steps} steps, tracking error {TrackingError():F3} rad.", this);

        if (n >= steps)
            Save();
    }

    float TrackingError()
    {
        if (controller == null || controller.CurrentTarget == null)
            return 0f;
        float[] target = controller.CurrentTarget;
        float[] state = agent.Observe();
        float sum = 0f;
        for (int i = 0; i < agent.JointCount; i++)
            sum += Mathf.Abs(state[i] - target[i]);
        return sum / agent.JointCount;
    }

    void Save()
    {
        saved = true;
        string dir = Path.Combine(Application.dataPath, outputSubdir);
        Directory.CreateDirectory(dir);
        string path = Path.Combine(dir, "rollout.csv");

        // Columns: q (6), qd (6), q* (6), q' (6), qd' (6) — 30 columns,
        // matching train.py's STATE + ACTION + STATE layout.
        var sb = new StringBuilder(capacity: states.Count * 128);
        for (int i = 0; i < states.Count; i++)
        {
            AppendRow(sb, states[i]);
            sb.Append(',');
            AppendRow(sb, actions[i]);
            sb.Append(',');
            AppendRow(sb, nextStates[i]);
            sb.Append('\n');
        }
        File.WriteAllText(path, sb.ToString());
        Debug.Log($"RolloutRecorder: saved {states.Count} transitions to {path}. " +
                  "You can stop Play now and run train.py.", this);
    }

    static void AppendRow(StringBuilder sb, float[] values)
    {
        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0)
                sb.Append(',');
            sb.Append(values[i].ToString("R", CultureInfo.InvariantCulture));
        }
    }
}
