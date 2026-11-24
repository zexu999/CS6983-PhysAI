using System;
using UnityEngine;


[DefaultExecutionOrder(-5)]
public class WorldModelController : MonoBehaviour
{
    public ArmAgent agent;
    public ArmJointController goals;
    public TextAsset controllerJson;

    [Range(0.0f, 1f)]
    public float actionSmoothing = 0.10f;

    public float maxDeltaPerStep = 0.05f;

    float[] W;
    float[] b;
    float[] jointLo;
    float[] jointHi;

    float[] features;
    float[] rawAction;
    float[] action;
    bool ready;
    bool hasAction;
    bool tookOverEmitActions;
    bool previousEmitActions;

    void Start()
    {
        if (agent == null)
            agent = GetComponent<ArmAgent>();
        if (goals == null)
            goals = GetComponent<ArmJointController>();
        if (agent == null || goals == null || controllerJson == null)
        {
            Debug.LogError("WorldModelController needs agent, goals, and controller.json.");
            enabled = false;
            return;
        }

        features = new float[agent.StateSize];
        rawAction = new float[agent.ActionSize];
        action = new float[agent.ActionSize];

        if (!Load(controllerJson.text))
        {
            enabled = false;
            return;
        }

        previousEmitActions = goals.emitActions;
        goals.emitActions = false;
        tookOverEmitActions = true;
        ready = true;
        Debug.Log(
            "WorldModelController ready (CMA-ES Controller C). " +
            "Goals come from ArmJointController; actions come from the learned linear policy.");
    }

    void OnEnable()
    {
        if (!ready || goals == null)
            return;

        previousEmitActions = goals.emitActions;
        goals.emitActions = false;
        tookOverEmitActions = true;
        hasAction = false;
    }

    void OnDisable()
    {
        if (tookOverEmitActions && goals != null)
        {
            goals.emitActions = previousEmitActions;
            tookOverEmitActions = false;
        }
        hasAction = false;
    }

    void FixedUpdate()
    {
        if (!ready || agent == null || goals == null || !agent.Ready)
            return;

        agent.CopyState(features);
        ActionFrom(features, rawAction);
        ApplySmoothing(rawAction, action);
        agent.SetAction(action);
    }

    public void ActionFrom(float[] state, float[] destination)
    {
        if (goals == null || goals.CurrentTarget == null)
            return;
        ActionFrom(state, goals.CurrentTarget, destination);
    }

    public void ActionFrom(float[] state, float[] goal, float[] destination)
    {
        if (!ready || state == null || goal == null || destination == null)
            return;

        // Feature layout (must match train_controller.py:
        // "goal_plus_W_q_minus_goal_and_qd"):
        //   features = [q - goal (ActionSize), qd (ActionSize)]
        //   action   = goal + W @ features + b, clamped to the joint limits.
        // W is stored row-major: W[i * StateSize + j].
        for (int i = 0; i < agent.ActionSize; i++)
        {
            float sum = goal[i] + b[i];
            for (int j = 0; j < agent.StateSize; j++)
            {
                float feature = j < agent.ActionSize ? state[j] - goal[j] : state[j];
                sum += W[i * agent.StateSize + j] * feature;
            }
            destination[i] = Mathf.Clamp(sum, jointLo[i], jointHi[i]);
        }
    }

    void ApplySmoothing(float[] raw, float[] smoothed)
    {
        if (!hasAction)
        {
            Array.Copy(raw, smoothed, raw.Length);
            hasAction = true;
            return;
        }
        // Mirrors train_controller.py's smooth_action(): exponential blend,
        // per-step delta cap, then joint-limit clamp.
        for (int i = 0; i < raw.Length; i++)
        {
            float blended = (1f - actionSmoothing) * smoothed[i] + actionSmoothing * raw[i];
            float delta = Mathf.Clamp(blended - smoothed[i], -maxDeltaPerStep, maxDeltaPerStep);
            smoothed[i] = Mathf.Clamp(smoothed[i] + delta, jointLo[i], jointHi[i]);
        }
    }

    bool Load(string json)
    {
        try
        {
            var data = JsonUtility.FromJson<ControllerFile>(json);
            if (data == null || data.W == null || data.b == null ||
                data.W.Length != agent.ActionSize * agent.StateSize ||
                data.b.Length != agent.ActionSize)
            {
                Debug.LogError(
                    "controller.json has unexpected W/b shapes for ActionSize=" +
                    agent.ActionSize + " StateSize=" + agent.StateSize + ".");
                return false;
            }

            W = data.W;
            b = data.b;
            jointLo = data.joint_lo != null && data.joint_lo.Length == agent.ActionSize
                ? data.joint_lo
                : CopyRangesLo();
            jointHi = data.joint_hi != null && data.joint_hi.Length == agent.ActionSize
                ? data.joint_hi
                : CopyRangesHi();
            return true;
        }
        catch (Exception exception)
        {
            Debug.LogException(exception);
            return false;
        }
    }

    float[] CopyRangesLo()
    {
        var result = new float[agent.ActionSize];
        for (int i = 0; i < agent.ActionSize; i++)
            result[i] = goals.jointRanges != null && i < goals.jointRanges.Length
                ? goals.jointRanges[i].x
                : -Mathf.PI;
        return result;
    }

    float[] CopyRangesHi()
    {
        var result = new float[agent.ActionSize];
        for (int i = 0; i < agent.ActionSize; i++)
            result[i] = goals.jointRanges != null && i < goals.jointRanges.Length
                ? goals.jointRanges[i].y
                : Mathf.PI;
        return result;
    }

    [Serializable]
    class ControllerFile
    {
        public string feature;
        public int state_size;
        public int action_size;
        public float[] W;
        public float[] b;
        public float[] joint_lo;
        public float[] joint_hi;
        public float[] home;
    }
}
