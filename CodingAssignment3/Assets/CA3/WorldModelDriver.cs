using System.Collections.Generic;
using Mujoco;
using Unity.InferenceEngine;
using UnityEngine;

public class WorldModelDriver : MonoBehaviour
{
    public ArmAgent agent;
    public ArmJointController controller;
    public WorldModelController policy;
    public ModelAsset modelAsset;
    public TextAsset normJson;
    public Transform ghostArm;
    public Transform ghostMarker;
    public LineRenderer forecast;
    public LineRenderer deviation;
    public Material ghostMaterial;
    public Material lineMaterial;

    public int horizon = 100;
    public int ghostInterval = 3000;
    [Range(0f, 0.25f)]
    public float softCorrection = 0.05f;
    public float maxJointError = 0.2f;
    public int forecastInterval = 10;
    public bool drawForecast = true;

    int FeatureSize => agent != null ? agent.StateSize + agent.ActionSize : 0;

    struct GhostPart
    {
        public MjBody source;
        public Transform ghost;
        public Matrix4x4 geomToMesh;
        public bool calibrated;
    }

    Worker worker;
    DynamicsNorm norm;
    Tensor<float> input;
    float[] features;
    float[] actionBuffer;
    float[] realState;
    double[] savedQ;
    GhostPart[] ghostParts = System.Array.Empty<GhostPart>();
    float[] ghostState;
    float[] simulated;
    float[] predicted;
    float[] forecastScratch;
    Vector3[] points;
    int ghostSteps;
    int forecastSteps;
    bool ready;
    bool ghostCalibrated;
    bool ownsGhostInstance;
    bool ownsForecast;
    bool ownsDeviation;
    Transform ghostPrefab;

    void Awake()
    {
        if (agent == null)
            agent = GetComponent<ArmAgent>();
        if (controller == null)
            controller = GetComponent<ArmJointController>();
        if (policy == null)
            policy = GetComponent<WorldModelController>();
        ghostPrefab = ghostArm;
    }

    void OnEnable()
    {
        if (ready)
            return;

        if (agent == null || modelAsset == null || normJson == null)
        {
            Debug.LogError("WorldModelDriver needs the agent, dynamics.onnx, and norm.json.");
            enabled = false;
            return;
        }

        if (!agent.Ready)
        {
            Debug.LogError("WorldModelDriver needs the agent to be ready.");
            enabled = false;
            return;
        }
            
        // Initialize arrays
        features = new float[FeatureSize];
        actionBuffer = new float[agent.ActionSize];
        realState = new float[agent.StateSize];
        savedQ = new double[agent.JointCount];
        ghostState = new float[agent.StateSize];
        simulated = new float[agent.StateSize];
        predicted = new float[agent.StateSize];
        forecastScratch = new float[agent.StateSize];

        // Load the norm.json file
        norm = JsonUtility.FromJson<DynamicsNorm>(normJson.text);
        if (norm == null ||
            norm.x_mean == null || norm.x_mean.Length != FeatureSize ||
            norm.y_mean == null || norm.y_mean.Length != agent.StateSize)
        {
            Debug.LogError(
                "norm.json does not match StateSize=" + agent.StateSize +
                " ActionSize=" + agent.ActionSize + ".");
            enabled = false;
            return;
        }

        // Create the worker
        worker = new Worker(ModelLoader.Load(modelAsset), BackendType.CPU);
        // Create the input tensor
        input = new Tensor<float>(new TensorShape(1, FeatureSize), features);
        // Create the visuals
        CreateVisuals();
        ready = true;
    }

    unsafe void FixedUpdate()
    {
        if (!ready || !agent.Ready || !MjScene.InstanceExists || MjScene.Instance.Data == null)
            return;

        // Copy the real state to the ghost state
        agent.CopyState(realState);
        // Reanchor the ghost if it has drifted too far from reality, or if
        // enough time has passed since the last reanchor. On the first step
        // the ghost state is all zeros, so the error check reanchors it
        // immediately.
        if (ghostSteps >= ghostInterval || JointError(ghostState, realState) > maxJointError)
            Reanchor();

        // Put the ghost state into the action buffer
        FillAction(ghostState, actionBuffer);
        // Predict the next state
        Predict(ghostState, actionBuffer, predicted);
        // Update the ghost state with the predicted state
        for (int i = 0; i < agent.StateSize; i++)
            ghostState[i] = predicted[i];
        // Blend the predicted state with the real state for some correction
        float keep = 1f - softCorrection;
        for (int i = 0; i < agent.StateSize; i++)
            ghostState[i] = keep * ghostState[i] + softCorrection * realState[i];

        ghostSteps++;

        // Pose the ghost meshes to the imagined state
        PlaceGhost(ghostState);
        // Show the forecast if the forecast interval is reached
        if (drawForecast && forecastSteps % Mathf.Max(1, forecastInterval) == 0)
            ShowForecast(ghostState);
        // Increment the forecast steps
        forecastSteps++;
        // Show the deviation
        ShowDeviation();
    }

    void CreateVisuals()
    {
        horizon = Mathf.Max(1, horizon);
        forecastInterval = Mathf.Max(1, forecastInterval);
        points = new Vector3[horizon + 1];
        // Build the ghost arm
        BuildGhostArm();

        // Create the forecast line
        if (forecast == null)
        {
            var go = new GameObject("ArmForecast");
            forecast = go.AddComponent<LineRenderer>();
            forecast.widthMultiplier = 0.01f;
            forecast.positionCount = points.Length;
            ownsForecast = true;
        }

        // Set the material for the forecast line
        if (lineMaterial != null)
            forecast.sharedMaterial = lineMaterial;
        forecast.enabled = drawForecast;
        // Set the position count for the forecast line
        forecast.positionCount = points.Length;

        // Create the deviation line
        if (deviation == null)
        {
            var go = new GameObject("ArmDeviation");
            deviation = go.AddComponent<LineRenderer>();
            deviation.widthMultiplier = 0.008f;
            deviation.positionCount = 2;
            ownsDeviation = true;
        }
        // Set the material for the deviation line
        if (lineMaterial != null)
            deviation.sharedMaterial = lineMaterial;
    }

    void BuildGhostArm()
    {
        // If the ghost prefab is not set, use the ghost arm
        if (ghostPrefab == null)
            ghostPrefab = ghostArm;

        // If the ghost prefab is not set, log an error
        if (ghostPrefab == null)
        {
            Debug.LogError(
                "WorldModelDriver needs a visual-only ghost arm prefab assigned to Ghost Arm.");
            ghostParts = System.Array.Empty<GhostPart>();
            return;
        }

        // Create a new instance of the ghost prefab in the scene
        if (!ghostPrefab.gameObject.scene.IsValid())
        {
            ghostArm = InstantiateGhostVisual(ghostPrefab.gameObject);
            ownsGhostInstance = true;
        }
        else
        {
            ghostArm = ghostPrefab;
            StripGhostMuJoCo(ghostArm.gameObject);
            ownsGhostInstance = false;
        }

        // Create the transform for the ghost end effector
        if (ghostMarker == null)
        {
            var marker = new GameObject("ArmGhostEE");
            marker.transform.SetParent(ghostArm, false);
            ghostMarker = marker.transform;
        }

        // Set the material for the ghost arm
        if (ghostMaterial != null)
        {
            foreach (var renderer in ghostArm.GetComponentsInChildren<MeshRenderer>(true))
            {
                renderer.sharedMaterial = ghostMaterial;
                renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                renderer.receiveShadows = false;
            }
        }

        // Map real MjBodies to the same-named transforms on the prefab.
        // Posing bodies (like MjBody.OnSyncState does) keeps every geom at
        // its imported local offset, so the ghost stays assembled.
        // GetComponentsInChildren is parent-first, so parents move first.
        var parts = new List<GhostPart>();
        foreach (var body in GetComponentsInChildren<MjBody>(true))
        {
            // Find the child transform with the same name as the MjBody
            // (the ghost root is renamed "base (Ghost)", so map it directly)
            var ghostTransform = body.transform == transform
                ? ghostArm
                : FindChild(ghostArm, body.name);
            if (ghostTransform == null)
                continue;

            // Add the ghost part to the list
            parts.Add(new GhostPart
            {
                source = body,
                ghost = ghostTransform,
                geomToMesh = Matrix4x4.identity,
                calibrated = false,
            });
        }

        // Set the ghost parts array to the list of ghost parts found
        ghostParts = parts.ToArray();
        ghostCalibrated = false;
        if (ghostParts.Length == 0)
            Debug.LogWarning(
                "WorldModelDriver found no name-matched geoms between the real arm and ghost prefab.");
    }

    static Transform InstantiateGhostVisual(GameObject source)
    {
        // Create a new instance of the ghost prefab in the scene
        bool restoreActive = source.activeSelf;
        if (restoreActive)
            source.SetActive(false);

        // Instantiate the ghost prefab
        var instance = Instantiate(source);
        // Set the name of the instance
        instance.name = source.name + " (Ghost)";

        if (restoreActive)
            source.SetActive(true);

        // Remove the MuJoCo components from the instance
        StripGhostMuJoCo(instance);
        // Set the instance to active
        instance.SetActive(true);
        return instance.transform;
    }

    static void StripGhostMuJoCo(GameObject root)
    {
        // Remove the CA3 scripts (ArmAgent, controllers, this driver) copied
        // with the prefab, so the ghost never runs or drives the real arm
        var ca3 = typeof(WorldModelDriver).Assembly;
        foreach (var behaviour in root.GetComponentsInChildren<MonoBehaviour>(true))
            if (behaviour != null && behaviour.GetType().Assembly == ca3)
                DestroyImmediate(behaviour);

        // Remove the MjMeshFilter components from the instance
        foreach (var meshFilter in root.GetComponentsInChildren<MjMeshFilter>(true))
            DestroyImmediate(meshFilter);

        // Remove the MjGlobalSettings components from the instance
        foreach (var settings in root.GetComponentsInChildren<MjGlobalSettings>(true))
            DestroyImmediate(settings);

        // Remove the MjComponent components from the instance
        foreach (var component in root.GetComponentsInChildren<MjComponent>(true))
            DestroyImmediate(component);

        // Disable the colliders on the instance
        foreach (var collider in root.GetComponentsInChildren<Collider>(true))
            collider.enabled = false;
    }

    static Transform FindChild(Transform root, string name)
    {
        if (root.name == name)
            return root;
        for (int i = 0; i < root.childCount; i++)
        {
            var found = FindChild(root.GetChild(i), name);
            if (found != null)
                return found;
        }
        return null;
    }

    void FillAction(float[] state, float[] destination)
    {
        // Default: hold the current joint goal (open-loop).
        if (controller != null && controller.CurrentTarget != null)
            controller.ActionFrom(destination);
        // When the learned policy is available it overrides the hold action
        // with the action it would take from this (imagined) state. If the
        // policy is not ready, ActionFrom leaves the hold action in place.
        if (policy != null && controller != null && controller.CurrentTarget != null)
            policy.ActionFrom(state, controller.CurrentTarget, destination);
    }

    void Reanchor()
    {
        for (int i = 0; i < agent.StateSize; i++)
            ghostState[i] = realState[i];
        ghostSteps = 0;
    }

    float JointError(float[] ghost, float[] real)
    {
        float sum = 0f;
        for (int i = 0; i < agent.ActionSize; i++)
        {
            float error = ghost[i] - real[i];
            sum += error * error;
        }
        return Mathf.Sqrt(sum / agent.ActionSize);
    }

    void Predict(float[] state, float[] action, float[] next)
    {
        // Build the normalized model input: [state, action].
        for (int i = 0; i < agent.StateSize; i++)
            features[i] = (state[i] - norm.x_mean[i]) / norm.x_std[i];
        for (int i = 0; i < agent.ActionSize; i++)
            features[agent.StateSize + i] =
                (action[i] - norm.x_mean[agent.StateSize + i]) / norm.x_std[agent.StateSize + i];

        // The input tensor shares the `features` array, so scheduling it
        // runs the model on exactly the values written above.
        worker.Schedule(input);
        var output = worker.PeekOutput() as Tensor<float>;
        float[] deltaNorm = output.DownloadToArray();

        // Denormalize the predicted state delta and step the state forward.
        for (int i = 0; i < agent.StateSize; i++)
            next[i] = state[i] + deltaNorm[i] * norm.y_std[i] + norm.y_mean[i];
    }

    unsafe void PlaceGhost(float[] state)
    {
        if (ghostParts.Length == 0)
            return;

        // Get the real arm's scene and data from the MjScene instance
        var scene = MjScene.Instance;
        var data = scene.Data;

        // If the ghost is not calibrated yet, calibrate the offsets
        if (!ghostCalibrated)
            CalibrateGhostOffsets(data);

        // Save the joint positions of the real arm
        for (int i = 0; i < agent.JointCount; i++)
            savedQ[i] = data->qpos[agent.joints[i].QposAddress];

        // Apply the joint state to the data
        ApplyJointStateNoRestore(state, scene, data);

        // Loop through the ghost parts and update the position and rotation of the ghost
        for (int i = 0; i < ghostParts.Length; i++)
        {
            // Get the ghost part
            var part = ghostParts[i];
            // Same conversion the plugin uses to sync the real bodies
            int id = part.source.MujocoId;
            part.ghost.SetPositionAndRotation(
                MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(data->xpos, id)),
                MjEngineTool.UnityQuaternion(MjEngineTool.MjQuaternionAtEntry(data->xquat, id)));
        }

        // Update the ghost marker position and rotation
        if (ghostMarker != null && agent.endEffectorBody != null)
        {
            // Get the position and rotation of the end effector from the data
            ghostMarker.position = agent.ToolPoint(data);
            // Get the rotation of the end effector from the data
            ghostMarker.rotation = MjEngineTool.UnityQuaternion(
                MjEngineTool.MjQuaternionAtEntry(data->xquat, agent.endEffectorBody.MujocoId));
        }

        // Restore the MuJoCo state to the real arm's state
        for (int i = 0; i < agent.JointCount; i++)
            data->qpos[agent.joints[i].QposAddress] = savedQ[i];
        // Run FK to update the positions of the geoms
        MujocoLib.mj_kinematics(scene.Model, data);
    }

    unsafe void CalibrateGhostOffsets(MujocoLib.mjData_* data)
    {
        // Loop through the ghost parts and calibrate the offsets from the real arm
        for (int i = 0; i < ghostParts.Length; i++)
        {
            // Get the ghost part
            var part = ghostParts[i];
            // Body poses come straight from MuJoCo, so no offset is needed
            part.geomToMesh = Matrix4x4.identity;
            // Set the calibrated flag to true
            part.calibrated = true;
            // Update the ghost part
            ghostParts[i] = part;
        }

        ghostCalibrated = true;
    }

    unsafe static Matrix4x4 GeomWorldMatrix(MujocoLib.mjData_* data, int geomId)
    {
        // Get the position and rotation of the geom from the data
        var position = MjEngineTool.UnityVector3(MjEngineTool.MjVector3AtEntry(data->geom_xpos, geomId));
        // Get the rotation of the geom from the data
        var rotation = MjEngineTool.UnityQuaternionFromMatrix(
            MjEngineTool.MjMatrixAtEntry(data->geom_xmat, geomId));
        // Return the transform of the geom
        return Matrix4x4.TRS(position, rotation, Vector3.one);
    }

    void ShowDeviation()
    {
        if (deviation == null || ghostMarker == null || agent.endEffector == null)
            return;
        // Set the positions of the deviation line
        deviation.SetPosition(0, ghostMarker.position);
        // Set the position of the end effector from the joint positions
        deviation.SetPosition(1, agent.endEffector.position);
    }

    unsafe void ShowForecast(float[] state)
    {
        if (forecast == null || agent.endEffectorBody == null)
            return;
        var scene = MjScene.Instance;
        var data = scene.Data;
        if (data == null)
            return;

        // Save the real joint positions; the forecast temporarily overwrites them.
        for (int i = 0; i < agent.JointCount; i++)
            savedQ[i] = data->qpos[agent.joints[i].QposAddress];

        // Roll the world model forward from the current imagined state,
        // recording the end effector position at every step.
        for (int i = 0; i < agent.StateSize; i++)
            forecastScratch[i] = state[i];
        points[0] = EndEffectorFromJointsNoRestore(forecastScratch, scene, data);
        for (int t = 0; t < horizon; t++)
        {
            FillAction(forecastScratch, actionBuffer);
            Predict(forecastScratch, actionBuffer, predicted);
            for (int i = 0; i < agent.StateSize; i++)
                forecastScratch[i] = predicted[i];
            points[t + 1] = EndEffectorFromJointsNoRestore(forecastScratch, scene, data);
        }

        // Restore the real joint positions and refresh the kinematics.
        for (int i = 0; i < agent.JointCount; i++)
            data->qpos[agent.joints[i].QposAddress] = savedQ[i];
        MujocoLib.mj_kinematics(scene.Model, data);

        forecast.positionCount = points.Length;
        forecast.SetPositions(points);
    }

    unsafe void ApplyJointStateNoRestore(float[] state, MjScene scene, MujocoLib.mjData_* data)
    {
        // Apply the world model's state to the data
        for (int i = 0; i < agent.JointCount; i++)
            data->qpos[agent.joints[i].QposAddress] = state[i];
        // Run FK to update the positions of the geoms
        MujocoLib.mj_kinematics(scene.Model, data);
    }

    unsafe Vector3 EndEffectorFromJointsNoRestore(float[] state, MjScene scene, MujocoLib.mjData_* data)
    {
        // Apply the world model's state to the data
        ApplyJointStateNoRestore(state, scene, data);
        // Get the tool point (attachment_site) from the data
        return agent.ToolPoint(data);
    }

    void OnDisable()
    {
        DisposeModel();
        TeardownVisuals();
        ready = false;
    }

    void OnDestroy()
    {
        DisposeModel();
        TeardownVisuals();
        ready = false;
    }

    void DisposeModel()
    {
        if (input != null)
        {
            input.Dispose();
            input = null;
        }
        if (worker != null)
        {
            worker.Dispose();
            worker = null;
        }
    }

    void TeardownVisuals()
    {
        if (ownsForecast && forecast != null)
        {
            Destroy(forecast.gameObject);
            forecast = null;
            ownsForecast = false;
        }
        else if (forecast != null)
        {
            forecast.enabled = false;
        }

        if (ownsDeviation && deviation != null)
        {
            Destroy(deviation.gameObject);
            deviation = null;
            ownsDeviation = false;
        }
        else if (deviation != null)
        {
            deviation.enabled = false;
        }

        if (ownsGhostInstance && ghostArm != null)
        {
            Destroy(ghostArm.gameObject);
            ghostArm = ghostPrefab;
            ghostMarker = null;
            ownsGhostInstance = false;
        }
        else if (ghostArm != null)
        {
            ghostArm.gameObject.SetActive(false);
        }

        ghostParts = System.Array.Empty<GhostPart>();
        ghostCalibrated = false;
    }
}

[System.Serializable]
public class DynamicsNorm
{
    public float[] x_mean;
    public float[] x_std;
    public float[] y_mean;
    public float[] y_std;
}
