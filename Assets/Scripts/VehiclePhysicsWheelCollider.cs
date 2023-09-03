using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace VehiclePhysics
{
    public class VehiclePhysicsWheelCollider : MonoBehaviour
    {
        [Header("Wheel")]
        [SerializeField] private float _radius = 0.3f;
        [SerializeField] private float _width = 0.2f;
        [SerializeField] private float _mass = 20f;
        [SerializeField] private AnimationCurve _gripCoefficientCurve;
        private float _gripCoefficient;
        private bool _useGripCurve;
        [Range(0.01f, 1)]
        [SerializeField] private float _staticFrictionCoefficient = .8f;
        private float _normalReaction;
        private float _circumference;
        public float Circumference { get { return _circumference; } }
        [SerializeField] private LayerMask _ignoreThis;

        [Header("Suspension")]
        [SerializeField] private float _suspensionDistance = 0.2f;
        [SerializeField] private float _springStrength = 30000f;
        [SerializeField] private float _springDamper = 4000f;
        private float _suspensionForce;
        private float _suspensionLength;
        private float _wheelContactDistance;

        [Header("Debug")]
        [SerializeField] private bool _neverHide;
        [SerializeField] private float _wheelSkidAngle;

        [System.NonSerialized]
        public float motorTorque;
        [System.NonSerialized]
        public float brakeTorque;
        [System.NonSerialized]
        public float steerAngle;

        private bool _wheelDidHit;
        private RaycastHit _wheelHit;

        private Rigidbody _rigidbody;

        public Vector3 wheelPosition => transform.position - (transform.up * _suspensionLength);

        private void Awake()
        {
            _rigidbody = GetComponentInParent<Rigidbody>();
            _suspensionLength = _suspensionDistance;
            _useGripCurve = true;
            _normalReaction = ((_rigidbody.mass / 4) + _mass) * -Physics.gravity.y;
            _circumference = (2 * _radius) * Mathf.PI;
        }

        private void Update()
        {
            ApplyTorque();
            ApplySteering();
        }

        private void ApplyTorque()
        {
            if (!_wheelDidHit) return;

            _rigidbody.AddForceAtPosition((transform.forward * motorTorque) + (-transform.forward * brakeTorque), transform.position);
        }

        private void ApplySteering()
        {
            transform.localEulerAngles = new Vector3(0f, steerAngle, 0f);
        }

        private void FixedUpdate()
        {
            //WheelRaycastV2();
            _wheelDidHit = WheelRaycast();

            if (_wheelDidHit)
            {
                Suspension();
                Grip();
                StaticFriction();
            }
        }

        private bool WheelRaycastV2()
        {
            Vector3 capsulePoint1 = transform.position + (transform.up * _radius * 2) - (transform.right * (_width * .5f));
            Vector3 capsulePoint2 = transform.position + (transform.up * _radius * 2) + (transform.right * (_width * .5f));

            RaycastHit[] hits = Physics.CapsuleCastAll(capsulePoint1, capsulePoint2, _radius, -transform.up, _suspensionDistance + (_radius * 2), ~_ignoreThis);
            bool oneHitInside = false;
            float minDistance = Mathf.Infinity;
            
            for (int i = 0; i < hits.Length; i++)
            {
                float localHit = transform.InverseTransformPoint(hits[i].point).x;
                if(localHit < (_width * .5f) && localHit > -(_width * .5f)) //A revoir
                {
                    //Hit inside the wheel
                    if (!oneHitInside)
                        oneHitInside = true;

                    float dist = Vector3.Distance(wheelPosition, hits[i].point);
                    if(dist < minDistance)
                    {
                        minDistance = dist;
                        _wheelHit = hits[i];
                        Debug.DrawLine(wheelPosition, hits[i].point, Color.green);
                    }
                    else
                    {
                        Debug.DrawLine(wheelPosition, hits[i].point, Color.blue);
                    }
                }
                else
                {
                    Debug.DrawLine(wheelPosition, hits[i].point, Color.red);
                }
            }

            return oneHitInside;
        }

        private bool WheelRaycast()
        {
            Vector3 capsulePoint1 = transform.position + (transform.up * _radius * 2) - (transform.right * (_width * .5f));
            Vector3 capsulePoint2 = transform.position + (transform.up * _radius * 2) + (transform.right * (_width * .5f));

            if (Physics.CapsuleCast(capsulePoint1, capsulePoint2, _radius, -transform.up, out _wheelHit, _suspensionDistance + (_radius * 2)))
            {
                float f = transform.InverseTransformPoint(_wheelHit.point).x;
                return f < _width && f > -_width;
            }
            else
            {
                return false;
            }
        
        }

        private void Suspension()
        {
            // world-space dire of the spring force
            Vector3 springDir = transform.up;

            // world-space velocity of this wheel
            Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(transform.position);

            // calculate offset from the raycast
            _wheelContactDistance = Mathf.Clamp((_suspensionDistance + (_radius * 2)) - _wheelHit.distance, 0f, _suspensionDistance);
            _suspensionLength = _suspensionDistance - _wheelContactDistance;

            // calculate velocity along the spring direction
            float vel = Vector3.Dot(springDir, wheelWorldVel);

            // calculate the magnitude of the dampened spring force
            _suspensionForce = (_wheelContactDistance * _springStrength) - (vel * _springDamper);

            // apply the force at the location of the wheel
            _rigidbody.AddForceAtPosition(springDir * _suspensionForce, transform.position);
        }

        private void Grip()
        {
            // world-space direction of the spring force
            Vector3 steeringDir = transform.right;

            // world-space velocity of the suspension;
            Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(wheelPosition);

            // velocity in the steering direction
            float steeringVel = Vector3.Dot(steeringDir, wheelWorldVel);

            _wheelSkidAngle = Mathf.Abs(90f - Vector3.Angle(steeringDir, wheelWorldVel));

            // the change in velocity
            float desiredVelChange = -steeringVel * (_useGripCurve ? _gripCoefficientCurve.Evaluate(_wheelSkidAngle / 90f) : _gripCoefficient);

            // turn change in velocity into an acceleration
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

            _rigidbody.AddForceAtPosition(steeringDir * _mass * desiredAccel, wheelPosition);
        }

        private void StaticFriction()
        {
            Vector3 direction = transform.forward;

            Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(wheelPosition);

            float velocity = Vector3.Dot(direction, wheelWorldVel);

            float currentForceApplied = Mathf.Abs(((_rigidbody.mass / 4) + _mass) * (velocity / Time.fixedDeltaTime));

            float fMax = _staticFrictionCoefficient * _normalReaction;

            if (fMax >= currentForceApplied)
            {
                float desiredAccel = -velocity / Time.fixedDeltaTime;
                _rigidbody.AddForceAtPosition(direction * _mass * desiredAccel, wheelPosition);
            }

        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (_neverHide | Selection.Contains(gameObject) || Selection.Contains(transform.root.gameObject))
            {
                //Draw suspension
                Gizmos.color = Color.yellow;
                Gizmos.DrawLine(transform.position, wheelPosition);

                //Draw wheel
                Handles.color = Color.green;
                Handles.DrawWireDisc(wheelPosition - (transform.right * _width * .5f), transform.right, _radius);
                Handles.DrawWireDisc(wheelPosition + (transform.right * _width * .5f), transform.right, _radius);

                if (_wheelDidHit)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireSphere(_wheelHit.point, .03f);
                }
            }
        }
#endif

        public void Setup(float radius, float mass, float gripFactor, float suspensionDistance, float springStrength, float springDamper)
        {
            _radius = radius;
            _mass = mass;
            _gripCoefficient = gripFactor;
            _suspensionDistance = suspensionDistance;
            _springStrength = springStrength;
            _springDamper = springDamper;
        }

        public void UpdateGripFactor(float value)
        {
            _useGripCurve = false;
            _gripCoefficient = value;
        }

        public void ResetGripFactor()
        {
            _useGripCurve = true;
        }
    }

    /* Previous version (Raycast approch)
    public class VehiclePhysicsWheelCollider : MonoBehaviour
    {
        [Header("Wheel")]
        [SerializeField] private float _radius = 0.3f;
        [SerializeField] private float _width = 0.2f;
        [SerializeField] private float _mass = 20f;
        [Range(0, 1)]
        [SerializeField] private float _gripFactor = 1f;
        private float _defaultGripFactor;
        [Range(0, 1)]
        [SerializeField] private float _forceAppPointDistance = 0f;
        private float _circumference;
        public float Circumference { get { return _circumference; } }

        [Header("Suspension")]
        [SerializeField] private float _suspensionDistance = 0.2f;
        [SerializeField] private float _springStrength = 30000f;
        [SerializeField] private float _springDamper = 4000f;
        private float _suspensionOffset;
        private float _suspensionForce;

        [Header("Debug")]
        [SerializeField] private bool _neverHide;

        [System.NonSerialized]
        public float motorTorque;
        [System.NonSerialized]
        public float brakeTorque;
        [System.NonSerialized]
        public float steerAngle;

        bool _rayDidHit;
        RaycastHit _wheelHit;

        private Rigidbody _rigidbody;

        public Vector3 WheelContact()
        {
            return WheelCenterPosition() - transform.up * Mathf.Lerp(_radius, 0, _forceAppPointDistance);
        }

        public Vector3 WheelCenterPosition()
        {
            return transform.position - transform.up * (_suspensionDistance - _suspensionOffset);
        }

        private void Awake()
        {
            _rigidbody = GetComponentInParent<Rigidbody>();
            if (!_rigidbody)
            {
                Debug.LogError("A wheel collider need a parent with a rigidbody component !");
            }

            _defaultGripFactor = _gripFactor;

            _circumference = (2 * _radius) * Mathf.PI;

            motorTorque = 0f;
            brakeTorque = 0f;
            steerAngle = 0f;
        }

        private void Update()
        {
            ApplyTorque();
            ApplySteering();
        }

        private void ApplyTorque()
        {
            if (_rayDidHit)
            {
                _rigidbody.AddForceAtPosition((transform.forward * motorTorque) + (-transform.forward * brakeTorque), WheelContact());
            }
        }

        private void ApplySteering()
        {
            transform.localEulerAngles = new Vector3(0f, steerAngle, 0f);
        }

        private void FixedUpdate()
        {
            WheelRaycast();
            Suspension();
            Grip();
        }

        private void WheelRaycast()
        {
            _rayDidHit = Physics.Raycast(transform.position, -transform.up, out _wheelHit, _suspensionDistance + _radius);
        }

        private void Suspension()
        {
            // suspension spring force
            if (_rayDidHit)
            {
                // world-space dire of the spring force
                Vector3 springDir = transform.up;

                // world-space velocity of this wheel
                Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(WheelContact());

                // calculate offset from the raycast
                _suspensionOffset = (_suspensionDistance + _radius) - _wheelHit.distance;

                // calculate velocity along the spring direction
                float vel = Vector3.Dot(springDir, wheelWorldVel);

                // calculate the magnitude of the dampened spring force
                _suspensionForce = (_suspensionOffset * _springStrength) - (vel * _springDamper);

                // apply the force at the location of the wheel
                _rigidbody.AddForceAtPosition(springDir * _suspensionForce, WheelContact());
            }
            else
            {
                _suspensionOffset = 0f;
            }
        }

        private void Grip()
        {
            // grip force
            if (_rayDidHit)
            {
                // world-space direction of the spring force
                Vector3 steeringDir = transform.right;

                // world-space velocity of the suspension;
                Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(WheelContact());

                // velocity in the steering direction
                float steeringVel = Vector3.Dot(steeringDir, wheelWorldVel);

                // the change in velocity
                float desiredVelChange = -steeringVel * _gripFactor;

                // turn change in velocity into an acceleration
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                _rigidbody.AddForceAtPosition(steeringDir * _mass * desiredAccel, WheelContact());
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (_neverHide | Selection.Contains(gameObject) || Selection.Contains(transform.root.gameObject))
            {
                Vector3 wheelPos = WheelCenterPosition();
                Vector3 wheelContact = WheelContact();

                //Draw suspension
                Handles.color = Color.yellow;
                Handles.DrawLine(transform.position, transform.position - transform.up * (_suspensionDistance - _suspensionOffset));

                Handles.color = Color.red;
                Handles.DrawLine(transform.position - transform.up * (_suspensionDistance - _suspensionOffset), transform.position - transform.up * _suspensionDistance);

                //Start suspension
                Handles.color = Color.yellow;
                Handles.DrawLine(transform.position - (transform.forward * 0.05f), transform.position + (transform.forward * 0.05f));

                //End suspension
                Handles.DrawLine(transform.position - (transform.up * (_suspensionDistance - _suspensionOffset)) - (transform.forward * 0.05f), transform.position - (transform.up * (_suspensionDistance - _suspensionOffset)) + (transform.forward * 0.05f));

                //Draw wheel
                Handles.color = Color.green;
                //Handles.DrawWireDisc(wheelPos, transform.right, _radius);

                //Draw wheel center
                Handles.DrawLine(wheelPos - (transform.forward * _radius), wheelPos + (transform.forward * _radius));

                //Draw suspsension force
                Handles.color = Color.blue;
                Handles.DrawLine(transform.position, transform.position + transform.up * Mathf.Clamp01(_suspensionForce));

                //Draw motor torque
                Handles.color = Color.blue;
                Handles.DrawLine(wheelContact, wheelContact + transform.forward * motorTorque * 0.001f);

                //Draw wheel contact
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(wheelContact, .03f);
            }
        }
#endif
        public void Setup(float radius, float mass, float gripFactor, float suspensionDistance, float springStrength, float springDamper)
        {
            _radius = radius;
            _mass = mass;
            _gripFactor = gripFactor;
            _suspensionDistance = suspensionDistance;
            _springStrength = springStrength;
            _springDamper = springDamper;
        }
    
        public void UpdateGripFactor(float value)
        {
            _gripFactor = value;
        }

        public void ResetGripFactor()
        {
            _gripFactor = _defaultGripFactor;
        }
    }
    */
}