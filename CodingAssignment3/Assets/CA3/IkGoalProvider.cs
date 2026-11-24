using Mujoco;
using UnityEngine;

// Cartesian IK (damped least-squares, numerical Jacobian) that produces
// joint goals for the learned WorldModelController.
//
// Forward kinematics is evaluated with MuJoCo itself: the solver writes a
// candidate q into the live mjData, runs mj_kinematics, and reads the end
// effector position, then restores the real qpos before returning. That way
// the solver always agrees with the simulated arm — no hand-derived
// kinematics and no Unity/MuJoCo frame conversions beyond MjEngineTool.
//
// Runs before the controllers (-15), so the joint goal is fresh when
// WorldModelController computes its action in the same physics step.
[DefaultExecutionOrder(-15)]
public class IkGoalProvider : MonoBehaviour
{
    public ArmAgent agent;
    public ArmJointController goals;
    public Transform target;

    [Tooltip("DLS iterations per physics step.")]
    public int maxIterations = 50;
    [Tooltip("Stop when the end effector is this close to the target (m).")]
    public float tolerance = 0.005f;
    [Tooltip("DLS damping lambda; larger = slower but steadier near singularities.")]
    public float damping = 0.05f;
    [Tooltip("Cap on the per-joint change in one iteration (rad).")]
    public float maxStepPerJoint = 0.1f;
    [Tooltip("Random restarts when the warm-started solve gets stuck above tolerance.")]
    public int maxRestarts = 4;
    [Tooltip("Keep IK solutions inside the joint box the rollout data covered " +
             "(home ± randomScale × half range), where the world model and " +
             "controller are valid. Off = full MJCF joint limits.")]
    public bool stayInDataRange = true;

    float[] solution;
    float[] scratchQ;
    float[] restartQ;
    float[,] jacobian;
    float[,] normalMatrix;
    double[] savedQ;
    bool ready;
    bool hasSolution;

    void Start()
    {
        if (agent == null)
            agent = GetComponent<ArmAgent>();
        if (goals == null)
            goals = GetComponent<ArmJointController>();
        if (agent == null || goals == null || target == null)
        {
            Debug.LogError("IkGoalProvider needs agent, goals, and a target Transform.", this);
            enabled = false;
            return;
        }

        solution = new float[agent.JointCount];
        scratchQ = new float[agent.JointCount];
        restartQ = new float[agent.JointCount];
        jacobian = new float[3, agent.JointCount];
        normalMatrix = new float[3, 3];
        savedQ = new double[agent.JointCount];
        ready = true;
    }

    unsafe void FixedUpdate()
    {
        if (!ready || !agent.Ready || target == null)
            return;
        if (!MjScene.InstanceExists || MjScene.Instance.Data == null)
            return;

        // Warm-start from the previous solution so the goal tracks the arm
        // continuously while the target is dragged. On the first run, seed
        // from the current joint angles instead.
        if (!hasSolution)
        {
            float[] state = agent.Observe();
            for (int i = 0; i < agent.JointCount; i++)
                solution[i] = state[i];
            ClampToRanges(solution);
            hasSolution = true;
        }

        float error = Solve(target.position, solution);

        // DLS can fall into a local minimum (e.g. taking the base the wrong
        // way around). If the warm-started solve is still far off, retry
        // from random seeds and keep the best result.
        for (int r = 0; r < maxRestarts && error > tolerance; r++)
        {
            RandomSeed(restartQ);
            float restartError = Solve(target.position, restartQ);
            if (restartError < error)
            {
                error = restartError;
                for (int i = 0; i < agent.JointCount; i++)
                    solution[i] = restartQ[i];
            }
        }

        goals.SetGoal(solution);

        // Once a second: IK residual (m) and joint tracking error (rad), so
        // IK problems and controller problems can be told apart.
        logTimer += Time.fixedDeltaTime;
        if (logTimer >= 1f)
        {
            logTimer = 0f;
            float[] s = agent.Observe();
            float track = 0f;
            for (int i = 0; i < agent.JointCount; i++)
                track = Mathf.Max(track, Mathf.Abs(s[i] - solution[i]));
            Vector3 ee = agent.endEffector != null ? agent.endEffector.position : Vector3.zero;
            Debug.Log($"IK residual {error * 100f:F1} cm | max joint tracking err {track:F3} rad | " +
                      $"EE-target dist {(ee - target.position).magnitude * 100f:F1} cm");
        }
    }

    float logTimer;

    // Uniform random configuration in the middle 70% of each joint range.
    void RandomSeed(float[] q)
    {
        for (int i = 0; i < q.Length; i++)
        {
            Range(i, out float lo, out float hi);
            float span = hi - lo;
            q[i] = lo + span * 0.15f + UnityEngine.Random.Range(0f, span * 0.7f);
        }
    }

    // DLS solve in place. Returns the final end-effector error (m).
    unsafe float Solve(Vector3 targetPos, float[] q)
    {
        var scene = MjScene.Instance;
        var data = scene.Data;

        // Save the real qpos; every FK evaluation below overwrites it.
        for (int i = 0; i < agent.JointCount; i++)
            savedQ[i] = data->qpos[agent.joints[i].QposAddress];

        const float eps = 1e-3f; // finite-difference step (rad)
        float error = float.MaxValue;
        for (int iter = 0; iter < maxIterations; iter++)
        {
            Vector3 ee = EndEffectorPosition(q, scene, data);
            Vector3 err = targetPos - ee;
            error = err.magnitude;
            if (error < tolerance)
                break;

            // Numerical 3x6 Jacobian: column j = d(EE position)/d(q_j).
            for (int j = 0; j < agent.JointCount; j++)
            {
                for (int k = 0; k < agent.JointCount; k++)
                    scratchQ[k] = q[k];
                scratchQ[j] += eps;
                Vector3 perturbed = EndEffectorPosition(scratchQ, scene, data);
                jacobian[0, j] = (perturbed.x - ee.x) / eps;
                jacobian[1, j] = (perturbed.y - ee.y) / eps;
                jacobian[2, j] = (perturbed.z - ee.z) / eps;
            }

            // DLS update: dq = J^T (J J^T + lambda^2 I)^-1 err.
            for (int r = 0; r < 3; r++)
            {
                for (int c = 0; c < 3; c++)
                {
                    float sum = 0f;
                    for (int j = 0; j < agent.JointCount; j++)
                        sum += jacobian[r, j] * jacobian[c, j];
                    normalMatrix[r, c] = sum + (r == c ? damping * damping : 0f);
                }
            }
            Vector3 y = Solve3x3(normalMatrix, err);
            for (int j = 0; j < agent.JointCount; j++)
            {
                float dq = jacobian[0, j] * y.x + jacobian[1, j] * y.y + jacobian[2, j] * y.z;
                q[j] += Mathf.Clamp(dq, -maxStepPerJoint, maxStepPerJoint);
            }
            ClampToRanges(q);
        }

        // Restore the real joint positions and refresh the kinematics.
        for (int i = 0; i < agent.JointCount; i++)
            data->qpos[agent.joints[i].QposAddress] = savedQ[i];
        MujocoLib.mj_kinematics(scene.Model, data);
        return error;
    }

    unsafe Vector3 EndEffectorPosition(float[] q, MjScene scene, MujocoLib.mjData_* data)
    {
        for (int i = 0; i < agent.JointCount; i++)
            data->qpos[agent.joints[i].QposAddress] = q[i];
        MujocoLib.mj_kinematics(scene.Model, data);
        return agent.ToolPoint(data);
    }

    void ClampToRanges(float[] q)
    {
        for (int i = 0; i < q.Length; i++)
        {
            Range(i, out float lo, out float hi);
            q[i] = Mathf.Clamp(q[i], lo, hi);
        }
    }

    // Allowed range for joint i: the MJCF limits, narrowed (by default) to
    // the same home ± span box ArmJointController sampled goals from while
    // collecting data.
    void Range(int i, out float lo, out float hi)
    {
        lo = -Mathf.PI; hi = Mathf.PI;
        if (goals.jointRanges != null && i < goals.jointRanges.Length)
        {
            lo = goals.jointRanges[i].x;
            hi = goals.jointRanges[i].y;
        }
        if (stayInDataRange && goals.homeTargets != null && i < goals.homeTargets.Length)
        {
            float span = (hi - lo) * 0.5f * goals.randomScale;
            float home = goals.homeTargets[i];
            lo = Mathf.Max(lo, home - span);
            hi = Mathf.Min(hi, home + span);
        }
    }

    // Cramer's rule for a 3x3 system. The normal matrix is symmetric
    // positive definite (JJ^T + lambda^2 I), so the determinant is > 0
    // whenever damping > 0.
    static Vector3 Solve3x3(float[,] a, Vector3 b)
    {
        float det = a[0, 0] * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1])
                  - a[0, 1] * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
                  + a[0, 2] * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
        if (Mathf.Abs(det) < 1e-9f)
            return Vector3.zero;
        float dx = b.x * (a[1, 1] * a[2, 2] - a[1, 2] * a[2, 1])
                 - a[0, 1] * (b.y * a[2, 2] - a[1, 2] * b.z)
                 + a[0, 2] * (b.y * a[2, 1] - a[1, 1] * b.z);
        float dy = a[0, 0] * (b.y * a[2, 2] - a[1, 2] * b.z)
                 - b.x * (a[1, 0] * a[2, 2] - a[1, 2] * a[2, 0])
                 + a[0, 2] * (a[1, 0] * b.z - b.y * a[2, 0]);
        float dz = a[0, 0] * (a[1, 1] * b.z - b.y * a[2, 1])
                 - a[0, 1] * (a[1, 0] * b.z - b.y * a[2, 0])
                 + b.x * (a[1, 0] * a[2, 1] - a[1, 1] * a[2, 0]);
        return new Vector3(dx / det, dy / det, dz / det);
    }
}
