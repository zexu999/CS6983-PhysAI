using System;
using UnityEngine;


[DefaultExecutionOrder(-10)]
public class ArmJointController : MonoBehaviour
{
    public ArmAgent agent;
    public float holdSeconds = 3f;

    public Vector2[] jointRanges;

    public float[] homeTargets;

    [Range(0f, 1f)]
    public float randomScale = 0.35f;

    public bool emitActions = true;

    // Step 3 check: keep sampling the same random joint goals used for data
    // collection while WorldModelController drives the arm (IK disabled).
    public bool verifyRandomGoals = false;

    // UR5e defaults from the Menagerie MJCF (radians). Used only when the
    // Inspector fields are left empty, so the scene works out of the box.
    // Ranges: ±2π for every joint except the elbow (±π).
    static readonly Vector2[] DefaultJointRanges =
    {
        new Vector2(-Mathf.PI * 2f, Mathf.PI * 2f), // shoulder_pan
        new Vector2(-Mathf.PI * 2f, Mathf.PI * 2f), // shoulder_lift
        new Vector2(-Mathf.PI, Mathf.PI),           // elbow
        new Vector2(-Mathf.PI * 2f, Mathf.PI * 2f), // wrist_1
        new Vector2(-Mathf.PI * 2f, Mathf.PI * 2f), // wrist_2
        new Vector2(-Mathf.PI * 2f, Mathf.PI * 2f), // wrist_3
    };

    // UR5e "home" keyframe from the MJCF.
    static readonly float[] DefaultHomeTargets =
    {
        -Mathf.PI / 2f, // shoulder_pan
        -Mathf.PI / 2f, // shoulder_lift
        Mathf.PI / 2f,  // elbow
        -Mathf.PI / 2f, // wrist_1
        -Mathf.PI / 2f, // wrist_2
        0f,             // wrist_3
    };

    float[] target;
    float hold;
    bool ready;

    public float[] CurrentTarget => target;

    void Start()
    {
        if (agent == null)
            agent = GetComponent<ArmAgent>();
        if (agent == null || !agent.Ready)
        {
            Debug.LogError("ArmJointController needs a ready ArmAgent.", this);
            enabled = false;
            return;
        }

        if (jointRanges == null || jointRanges.Length != agent.ActionSize)
            jointRanges = (Vector2[])DefaultJointRanges.Clone();
        if (homeTargets == null || homeTargets.Length != agent.ActionSize)
            homeTargets = (float[])DefaultHomeTargets.Clone();

        target = new float[agent.ActionSize];
        PlaceTarget(home: true);
        hold = holdSeconds;
        ready = true;
    }

    void FixedUpdate()
    {
        if (!ready || !agent.Ready)
            return;

        if (!emitActions)
        {
            // WorldModelController drives the arm. Goals come from
            // IkGoalProvider, or from random sampling in verify mode.
            if (verifyRandomGoals)
            {
                hold -= Time.fixedDeltaTime;
                if (hold <= 0f)
                {
                    PlaceTarget(home: false, apply: false);
                    hold = holdSeconds;
                }
            }
            return;
        }

        hold -= Time.fixedDeltaTime;
        if (hold <= 0f)
        {
            PlaceTarget(home: false);
            hold = holdSeconds;
        }
        agent.SetAction(target);
    }

    public void ActionFrom(float[] destination)
    {
        if (target == null || destination == null)
            return;
        Array.Copy(target, destination, Math.Min(target.Length, destination.Length));
    }

    public void PlaceTarget(bool home, bool apply = true)
    {
        if (target == null || jointRanges == null || homeTargets == null)
            return;

        if (home)
        {
            Array.Copy(homeTargets, target, target.Length);
        }
        else
        {
            // Random joint goal around home. Same sampling as
            // train_controller.py's sample_goal(), so the controller is
            // trained and verified on the same goal distribution that the
            // rollout data was collected with.
            for (int i = 0; i < target.Length; i++)
            {
                float span = (jointRanges[i].y - jointRanges[i].x) * 0.5f * randomScale;
                float value = homeTargets[i] + UnityEngine.Random.Range(-span, span);
                target[i] = Mathf.Clamp(value, jointRanges[i].x, jointRanges[i].y);
            }
        }
        if (apply)
            agent?.SetAction(target);
    }

    // Live-tracking entry point: IkGoalProvider writes the solved joint goal
    // here. The goal is clamped to the joint ranges; the learned
    // WorldModelController turns it into actions.
    public void SetGoal(float[] goal)
    {
        if (target == null || goal == null)
            return;
        for (int i = 0; i < target.Length && i < goal.Length; i++)
            target[i] = jointRanges != null && i < jointRanges.Length
                ? Mathf.Clamp(goal[i], jointRanges[i].x, jointRanges[i].y)
                : goal[i];
    }
}
