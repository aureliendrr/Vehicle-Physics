using UnityEditor;
using UnityEngine;

namespace VehiclePhysics
{
    public class VehiclePhysicsEditorWindow : EditorWindow
    {
        [MenuItem("Vehicle Physics/Vehicle Creator")]
        public static void ShowExample()
        {
            VehiclePhysicsEditorWindow wnd = GetWindow<VehiclePhysicsEditorWindow>();
            wnd.titleContent = new GUIContent("Vehicle Creator");
        }

        private bool _error = false;

        private string _vehicleName = "New Vehicle";
        private float _vehicleMass = 1300f;

        private float _wheelsRadius = .3f;
        private float _wheelsMass = 20f;
        private float _wheelsSuspensionDistance = .2f;
        private float _wheelsSpringStrength = 35000f;
        private float _wheelsSpringDamper = 4500f;

        private float _frontWheelsGripFactor = .7f;
        private float _rearWheelsGripFactor = .4f;

        public void OnGUI()
        {
            _error = false;

            EditorGUILayout.LabelField("Vehicle", new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontSize = 14, fontStyle = FontStyle.Bold }, GUILayout.ExpandWidth(true));
            EditorGUILayout.Space();

            _vehicleName = EditorGUILayout.TextField("Name", _vehicleName);
            if (_vehicleName.Length <= 0) ShowError("The vehicle name cannot be empty !");

            _vehicleMass = EditorGUILayout.FloatField("Mass", _vehicleMass);
            if (_vehicleMass <= 0) ShowError("The vehicle mass cannot be negative or null !");

            EditorGUILayout.Space(15f);
            EditorGUILayout.LabelField("Wheels", new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontSize = 14, fontStyle = FontStyle.Bold }, GUILayout.ExpandWidth(true));
            EditorGUILayout.Space();

            _wheelsRadius = EditorGUILayout.FloatField("Radius", _wheelsRadius);
            if (_wheelsRadius <= 0) ShowError("The wheel radius cannot be negative or null !");

            _wheelsMass = EditorGUILayout.FloatField("Mass", _wheelsMass);
            if (_wheelsMass <= 0) ShowError("The wheel mass cannot be negative or null !");

            _wheelsSuspensionDistance = EditorGUILayout.FloatField("Suspension Distance", _wheelsSuspensionDistance);
            if (_wheelsSuspensionDistance < 0) ShowError("The wheel suspension distance cannot be negative !");

            _wheelsSpringStrength = EditorGUILayout.FloatField("Spring Strength", _wheelsSpringStrength);
            if (_wheelsSpringStrength < 0) ShowError("The wheel spring strength cannot be negative !");

            _wheelsSpringDamper = EditorGUILayout.FloatField("Spring Damper", _wheelsSpringDamper);
            if (_wheelsSpringDamper < 0) ShowError("The wheel spring damper cannot be negative !");

            _frontWheelsGripFactor = EditorGUILayout.Slider("Front Wheels Grip Factor", _frontWheelsGripFactor, 0f, 1f);
            if (_frontWheelsGripFactor < 0) ShowError("The front wheels grip factor cannot be negative !");

            _rearWheelsGripFactor = EditorGUILayout.Slider("Rear Wheels Grip Factor", _rearWheelsGripFactor, 0f, 1f);
            if (_rearWheelsGripFactor < 0) ShowError("The rear wheels grip factor cannot be negative !");

            EditorGUILayout.Space(15f);
            EditorGUI.BeginDisabledGroup(_error);
            if (GUILayout.Button("Create Vehicle"))
            {
                CreateNewVehicle();
                Close();
            }
            EditorGUI.EndDisabledGroup();
        }

        private void ShowError(string message)
        {
            EditorGUILayout.HelpBox(message, MessageType.Error);
            _error = true;
        }

        private void CreateNewVehicle()
        {
            GameObject vehicle = new GameObject(_vehicleName);
            Rigidbody vehicleRb = vehicle.AddComponent<Rigidbody>();
            vehicleRb.mass = _vehicleMass;
            vehicleRb.drag = 0f;
            vehicleRb.angularDrag = 0f;

            VehiclePhysicsController vehiclePhysicsController = vehicle.AddComponent<VehiclePhysicsController>();

            GameObject com = new GameObject("COM");
            com.transform.SetParent(vehicle.transform);
            com.transform.localPosition = new Vector3(0f, 0.3f, 0f);
            vehiclePhysicsController.SetCOM(com.transform);

            GameObject graphicsParent = new GameObject("Graphics");
            graphicsParent.transform.SetParent(vehicle.transform);

            GameObject simpleCube = GameObject.CreatePrimitive(PrimitiveType.Cube);
            simpleCube.transform.SetParent(graphicsParent.transform);
            simpleCube.transform.SetLocalPositionAndRotation(Vector3.up * 0.5f, Quaternion.identity);
            simpleCube.transform.localScale = new Vector3(1, 0.5f, 2f);

            GameObject wheelsColliderParent = new GameObject("Wheels Colliders");
            wheelsColliderParent.transform.SetParent(vehicle.transform);

            VehiclePhysicsWheelCollider frWheelCollider = new GameObject("FR").AddComponent<VehiclePhysicsWheelCollider>();
            frWheelCollider.Setup(_wheelsRadius, _wheelsMass, _frontWheelsGripFactor, _wheelsSuspensionDistance, _wheelsSpringStrength, _wheelsSpringDamper);
            frWheelCollider.transform.SetParent(wheelsColliderParent.transform);
            frWheelCollider.transform.SetLocalPositionAndRotation(new Vector3(0.6f, 0.4f, 0.75f), Quaternion.identity);
            vehiclePhysicsController.AddWheel(frWheelCollider, false, true);

            VehiclePhysicsWheelCollider flWheelCollider = new GameObject("FL").AddComponent<VehiclePhysicsWheelCollider>();
            flWheelCollider.Setup(_wheelsRadius, _wheelsMass, _frontWheelsGripFactor, _wheelsSuspensionDistance, _wheelsSpringStrength, _wheelsSpringDamper);
            flWheelCollider.transform.SetParent(wheelsColliderParent.transform);
            flWheelCollider.transform.SetLocalPositionAndRotation(new Vector3(-0.6f, 0.4f, 0.75f), Quaternion.identity);
            vehiclePhysicsController.AddWheel(flWheelCollider, false, true);

            VehiclePhysicsWheelCollider rrWheelCollider = new GameObject("RR").AddComponent<VehiclePhysicsWheelCollider>();
            rrWheelCollider.Setup(_wheelsRadius, _wheelsMass, _rearWheelsGripFactor, _wheelsSuspensionDistance, _wheelsSpringStrength, _wheelsSpringDamper);
            rrWheelCollider.transform.SetParent(wheelsColliderParent.transform);
            rrWheelCollider.transform.SetLocalPositionAndRotation(new Vector3(0.6f, 0.4f, -0.75f), Quaternion.identity);
            vehiclePhysicsController.AddWheel(rrWheelCollider, true, false);

            VehiclePhysicsWheelCollider rlWheelCollider = new GameObject("RL").AddComponent<VehiclePhysicsWheelCollider>();
            rlWheelCollider.Setup(_wheelsRadius, _wheelsMass, _rearWheelsGripFactor, _wheelsSuspensionDistance, _wheelsSpringStrength, _wheelsSpringDamper);
            rlWheelCollider.transform.SetParent(wheelsColliderParent.transform);
            rlWheelCollider.transform.SetLocalPositionAndRotation(new Vector3(-0.6f, 0.4f, -0.75f), Quaternion.identity);
            vehiclePhysicsController.AddWheel(rlWheelCollider, true, false);

            GameObject wheelsVisualParent = new GameObject("Wheels Visuals");
            wheelsVisualParent.transform.SetParent(vehicle.transform);

            GameObject cameraLookAt = new GameObject("Camera Look At");
            cameraLookAt.transform.SetParent(vehicle.transform);
            cameraLookAt.transform.SetLocalPositionAndRotation(Vector3.up * 1.25f, Quaternion.identity);

            vehicle.transform.SetPositionAndRotation(Vector3.up * .25f, Quaternion.identity);

            //Camera if exist
            Cinemachine.CinemachineFreeLook cinemachineFreeLook = FindFirstObjectByType<Cinemachine.CinemachineFreeLook>();
            if (cinemachineFreeLook)
            {
                cinemachineFreeLook.Follow = vehicle.transform;
                cinemachineFreeLook.LookAt = cameraLookAt.transform;
            }

            Selection.activeObject = vehicle;
        }
    }
}
