using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehiclePhysics
{
    [RequireComponent(typeof(Rigidbody))]
    public class VehiclePhysicsController : MonoBehaviour
    {
        [Header("Wheels")]
        [SerializeField] private List<Wheel> _wheels = new List<Wheel>();

        [Header("Settings")]
        [SerializeField] private Transform _centerOfMass;
        [SerializeField] private float _downForce;
        [Space(5f)]
        [SerializeField] private float _maxSpeed = 100f;
        [SerializeField] private float _power = 280f;
        [SerializeField] private AnimationCurve _powerSpeed = new AnimationCurve(new Keyframe(0, 1), new Keyframe(1, 1));
        [Space(5f)]
        [SerializeField] private float _handbrakeGripFactor = .1f;
        [Space(5f)]
        [SerializeField] private float _maxSteerAngle = 40f;
        [SerializeField] private float _steeringSpeed = .2f;

        [Header("Inputs")]
        [SerializeField] private float _gazInput;
        [SerializeField] private float _brakeInput;
        [SerializeField] private float _steeringInput;
        [SerializeField] private bool _handbrakeInput;
        public bool Handbrake { get { return _handbrakeInput; } }

        [Header("Infos")]
        [SerializeField] private float _steeringAngle;
        public float SteeringAngle { get { return _steeringAngle; } }
        [SerializeField] private float _speed;
        public float SpeedInKPH { get { return _speed * 3.6f; } }

        private Rigidbody _rigidbody;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();

            if (_centerOfMass)
                _rigidbody.centerOfMass = _centerOfMass.localPosition;
            else
                _rigidbody.ResetCenterOfMass();
        }

        private void Update()
        {
            GetInputs();
            ApplyWheelsPhysic();
            ApplyWheelsVisual();
        }

        private void GetInputs()
        {
            _gazInput = Input.GetAxis("Vertical");
            _steeringInput = Input.GetAxis("Horizontal");
            _handbrakeInput = Input.GetKey(KeyCode.Space);
        }

        private void ApplyWheelsPhysic()
        {
            _speed = Vector3.Dot(transform.forward, _rigidbody.velocity);
            _steeringAngle = Mathf.Lerp(_steeringAngle, _steeringInput * _maxSteerAngle, _steeringSpeed * Time.deltaTime);

            foreach (Wheel wheel in _wheels)
            {
                if (wheel.drive)
                {
                    if (_handbrakeInput)
                    {
                        wheel.collider.motorTorque = 0f;
                        wheel.collider.UpdateGripFactor(_handbrakeGripFactor);
                    }
                    else
                    {
                        wheel.collider.motorTorque = _gazInput * _powerSpeed.Evaluate(_speed / _maxSpeed) * _power;
                        wheel.collider.ResetGripFactor();
                    }
                }

                if (wheel.steer)
                {
                    wheel.collider.steerAngle = _steeringAngle;
                }
            }
        }

        private void ApplyWheelsVisual()
        {
            foreach (Wheel wheel in _wheels)
            {
                //Apply steering angle
                if (wheel.visualSteering)
                {
                    wheel.visualSteering.position = wheel.collider.wheelPosition;
                    if (wheel.steer)
                        wheel.visualSteering.localRotation = Quaternion.Euler(0, _steeringAngle, 0f);
                }

                //Apply rotation
                if (wheel.visualRotation)
                {
                    if(_handbrakeInput & wheel.drive)
                    {
                        wheel.visualRotation.localRotation = Quaternion.Euler(Vector3.zero);
                    }
                    else
                    {
                        Vector3 forwardSpeed = transform.InverseTransformDirection(_rigidbody.velocity);
                        float rotationPerSecond = forwardSpeed.z / wheel.collider.Circumference;
                        wheel.visualRotation.localRotation *= Quaternion.Euler(rotationPerSecond, 0f, 0f);
                    }
                }
            }
        }

        private void FixedUpdate()
        {
            ApplyDownForce();
        }

        private void ApplyDownForce()
        {
            _rigidbody.AddForce(-transform.up * _downForce * _rigidbody.velocity.magnitude);
        }

        public void AddWheel(VehiclePhysicsWheelCollider wheel, bool drive, bool steer)
        {
            _wheels.Add(new Wheel(wheel, drive, steer));
        }

        public void SetCOM(Transform com)
        {
            _centerOfMass = com;
        }
    }

    [System.Serializable]
    public class Wheel
    {
        public VehiclePhysicsWheelCollider collider;
        public Transform visualSteering;
        public Transform visualRotation;
        public bool drive;
        public bool steer;

        public Wheel(VehiclePhysicsWheelCollider collider, bool drive, bool steer)
        {
            this.collider = collider;
            this.drive = drive;
            this.steer = steer;
        }
    }
}