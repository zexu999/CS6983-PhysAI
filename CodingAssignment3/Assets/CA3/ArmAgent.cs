using System;
using Mujoco;
using UnityEngine;

// Runs before every other CA3 script, so the agent is already initialized
// when ArmJointController.Start and WorldModelDriver.OnEnable need it.
[DefaultExecutionOrder(-20)]
public class ArmAgent : MonoBehaviour
{
    public int JointCount;
    public int StateSize;
    public int ActionSize;

    public MjActuator gripper;
    public MjBody endEffectorBody;
    public Transform endEffector;
    public MjHingeJoint[] joints;
    public MjActuator[] actuators;

    // UR5e element names from the MuJoCo Menagerie MJCF
    // (universal_robots_ur5e/ur5e.xml). The joint order here defines the
    // state/action ordering used by every other script in this assignment.
    static readonly string[] JointNames =
    {
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    };

    static readonly string[] ActuatorNames =
    {
        "shoulder_pan",
        "shoulder_lift",
        "elbow",
        "wrist_1",
        "wrist_2",
        "wrist_3",
    };

    const string EndEffectorBodyName = "wrist_3_link";
    const string EndEffectorSiteName = "attachment_site";

    float[] action;
    float[] state;
    bool ready;

    // attachment_site offset in the wrist_3_link frame (Unity axes). The
    // site sits 10 cm past the body origin, so IK and the ghost marker use
    // this tool point rather than the body origin.
    public Vector3 toolOffsetLocal;

    public float[] CurrentAction => action;

    // Tool point (attachment_site) from the current mjData kinematics.
    public unsafe Vector3 ToolPoint(MujocoLib.mjData_* data)
    {
        int id = endEffectorBody.MujocoId;
        Vector3 p = MjEngineTool.UnityVector3(data->xpos + 3 * id);
        Quaternion r = MjEngineTool.UnityQuaternion(MjEngineTool.MjQuaternionAtEntry(data->xquat, id));
        return p + r * toolOffsetLocal;
    }
    public bool Ready => ready;

    void Awake()
    {
        InitializeJointsAndActuators();
    }

    void FixedUpdate()
    {
        // Keep the cached observation fresh. Control is applied immediately
        // in SetAction, so there is nothing to do here per step.
        if (ready)
            Observe();
    }

    public void InitializeJointsAndActuators()
    {
        // Resolve joints/actuators by MJCF name when they were not wired in
        // the Inspector. Inspector wiring (if present) is trusted as-is.
        if (joints == null || joints.Length == 0)
        {
            var found = new MjHingeJoint[JointNames.Length];
            for (int i = 0; i < JointNames.Length; i++)
                found[i] = FindComponent<MjHingeJoint>(JointNames[i]);
            joints = found;
        }
        if (actuators == null || actuators.Length == 0 || Array.Exists(actuators, a => a == null))
        {
            // The MuJoCo importer puts actuators in a sibling "actuators"
            // object, not under the arm root, so search the whole scene.
            var all = FindObjectsByType<MjActuator>(FindObjectsInactive.Include, FindObjectsSortMode.None);
            var found = new MjActuator[ActuatorNames.Length];
            for (int i = 0; i < ActuatorNames.Length; i++)
                found[i] = Array.Find(all, a => a.name == ActuatorNames[i]);
            actuators = found;
        }
        if (endEffectorBody == null)
            endEffectorBody = FindComponent<MjBody>(EndEffectorBodyName);
        if (endEffector == null)
        {
            // Prefer the attachment site (the true tool point). The plugin
            // imports sites as plain GameObjects, so look it up by name.
            // Fall back to the wrist link transform.
            var site = FindChildTransform(transform, EndEffectorSiteName);
            endEffector = site != null
                ? site
                : (endEffectorBody != null ? endEffectorBody.transform : null);
        }
        if (endEffector != null && endEffectorBody != null)
        {
            var bt = endEffectorBody.transform;
            toolOffsetLocal = Quaternion.Inverse(bt.rotation) * (endEffector.position - bt.position);
        }
        // The UR5e has no gripper; `gripper` intentionally stays null.

        bool ok = true;
        for (int i = 0; i < joints.Length; i++)
        {
            if (joints[i] == null)
            {
                string label = i < JointNames.Length ? JointNames[i] : $"joint[{i}]";
                Debug.LogError($"ArmAgent: joint '{label}' not found under {name}.", this);
                ok = false;
            }
        }
        for (int i = 0; i < actuators.Length; i++)
        {
            if (actuators[i] == null)
            {
                string label = i < ActuatorNames.Length ? ActuatorNames[i] : $"actuator[{i}]";
                Debug.LogError($"ArmAgent: actuator '{label}' not found under {name}.", this);
                ok = false;
            }
        }
        if (endEffectorBody == null)
        {
            Debug.LogError($"ArmAgent: end effector body '{EndEffectorBodyName}' not found under {name}.", this);
            ok = false;
        }

        JointCount = joints.Length;
        ActionSize = JointCount;
        StateSize = 2 * JointCount;
        action = new float[ActionSize];
        state = new float[StateSize];
        ready = ok;
        if (ok)
            Debug.Log($"ArmAgent ready: {JointCount} joints, state {StateSize}, action {ActionSize}.", this);
    }

    public void SetAction(float[] targets)
    {
        if (!ready || targets == null)
            return;
        int n = Math.Min(targets.Length, action.Length);
        Array.Copy(targets, action, n);
        // Apply immediately so the new target is already in the actuator
        // Control fields when MjScene syncs them into MuJoCo this step.
        ApplyAction();
    }

    public void ApplyAction()
    {
        if (!ready)
            return;
        for (int i = 0; i < actuators.Length && i < action.Length; i++)
        {
            if (actuators[i] != null)
                actuators[i].Control = action[i];
        }
    }

    public float[] Observe()
    {
        if (!ready)
            return state;
        for (int i = 0; i < JointCount; i++)
        {
            // RawConfiguration is the unnormalized joint angle in radians.
            // (Configuration is degrees wrapped to [0, 360), not usable here.)
            state[i] = (float)joints[i].RawConfiguration;
            state[JointCount + i] = joints[i].Velocity;
        }
        return state;
    }

    public void CopyState(float[] destination)
    {
        Observe();
        Array.Copy(state, destination, Math.Min(state.Length, destination.Length));
    }

    private T FindComponent<T>(string name) where T : Component
    {
        foreach (var component in GetComponentsInChildren<T>(true))
        {
            if (component.name == name)
                return component;
        }
        return null;
    }

    static Transform FindChildTransform(Transform root, string name)
    {
        if (root.name == name)
            return root;
        for (int i = 0; i < root.childCount; i++)
        {
            var found = FindChildTransform(root.GetChild(i), name);
            if (found != null)
                return found;
        }
        return null;
    }
}
