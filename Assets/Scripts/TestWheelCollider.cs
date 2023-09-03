using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class TestWheelCollider : MonoBehaviour
{
    [Header("Wheel")]
    [SerializeField] private float _radius = 0.3f;
    [SerializeField] private float _width = 0.2f;
    [SerializeField] private float _mass = 20f;
    [Range(0, 1)]
    [SerializeField] private float _gripFactor = 1f;

    [Header("Suspension")]
    [SerializeField] private float _suspensionMaxLength = .2f;
    [SerializeField] private float _springStrength = 30000f;
    [SerializeField] private float _springDamper = 4000f;
    [Space(5f)]
    [SerializeField] private float _suspensionForce;
    [SerializeField] private float _suspensionLength;
    [SerializeField] private float _wheelContactDistance;

    [Header("Debug")]
    [SerializeField] private bool _neverHide;

    private bool _wheelDidHit;
    private RaycastHit _wheelHit;

    private Rigidbody _rigidbody;

    private Vector3 wheelPosition => transform.position - (transform.up * _suspensionLength);

    private void Awake()
    {
        _rigidbody = GetComponentInParent<Rigidbody>();
        _suspensionLength = _suspensionMaxLength;
    }

    private void FixedUpdate()
    {
        _wheelDidHit = WheelRaycast();

        Suspension();
        Grip();
    }

    //V1
    private bool WheelRaycast()
    {
        Vector3 capsulePoint1 = transform.position + (transform.up * _radius * 2) - (transform.right * (_width * .5f));
        Vector3 capsulePoint2 = transform.position + (transform.up * _radius * 2) + (transform.right * (_width * .5f));

        if (Physics.CapsuleCast(capsulePoint1, capsulePoint2, _radius, -transform.up, out _wheelHit, _suspensionMaxLength + (_radius * 2)))
        {
            float f = transform.InverseTransformPoint(_wheelHit.point).x;
            return f < _width && f > -_width;
        }
        else
        {
            return false;
        }
    }

    //V-nop
    /*
    private bool WheelRaycast()
    {
        Vector3 capsulePoint1 = transform.position + (transform.up * _radius * 2) - (transform.right * (_widthByTwo));
        Vector3 capsulePoint2 = transform.position + (transform.up * _radius * 2) + (transform.right * (_widthByTwo));

        RaycastHit[] hits = Physics.CapsuleCastAll(capsulePoint1, capsulePoint2, _radius, -transform.up, _suspensionMaxLength + (_radius * 2));

        if(hits.Length > 0)
        {
            bool oneHitInside = false;
            float minDistance = Mathf.Infinity;

            for (int i = 0; i < hits.Length; i++)
            {
                Debug.DrawLine(transform.position, hits[i].point, Color.red);

                //Hit inside the wheel (not side spheres of the capsule
                Vector3 localPoint = transform.InverseTransformPoint(hits[i].point);
                if(localPoint.x < _widthByTwo && localPoint.x > -_widthByTwo)
                {
                    if(!oneHitInside)
                        oneHitInside = true;

                    if (hits[i].distance < minDistance)
                    {
                        _wheelHit = hits[i];
                        minDistance = hits[i].distance;
                    }
                }
            }

            if(oneHitInside)
                Debug.DrawLine(transform.position, _wheelHit.point, Color.green);

            return oneHitInside;
        }
        else
        {
            return false;
        }
    }
    */

    private void Suspension()
    {
        if (_wheelDidHit)
        {
            // world-space dire of the spring force
            Vector3 springDir = transform.up;

            // world-space velocity of this wheel
            Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(transform.position);

            // calculate offset from the raycast
            _wheelContactDistance = Mathf.Clamp((_suspensionMaxLength + (_radius * 2)) - _wheelHit.distance, 0f, _suspensionMaxLength);
            _suspensionLength = _suspensionMaxLength - _wheelContactDistance;

            // calculate velocity along the spring direction
            float vel = Vector3.Dot(springDir, wheelWorldVel);

            // calculate the magnitude of the dampened spring force
            _suspensionForce = (_wheelContactDistance * _springStrength) - (vel * _springDamper);

            // apply the force at the location of the wheel
            _rigidbody.AddForceAtPosition(springDir * _suspensionForce, transform.position);
        }
        else
        {
            _wheelContactDistance = 0f;
        }
    }

    private void Grip()
    {
        // grip force
        if (_wheelDidHit)
        {
            // world-space direction of the spring force
            Vector3 steeringDir = transform.right;

            // world-space velocity of the suspension;
            Vector3 wheelWorldVel = _rigidbody.GetPointVelocity(transform.position);

            // velocity in the steering direction
            float steeringVel = Vector3.Dot(steeringDir, wheelWorldVel);

            // the change in velocity
            float desiredVelChange = -steeringVel * _gripFactor;

            // turn change in velocity into an acceleration
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

            _rigidbody.AddForceAtPosition(steeringDir * _mass * desiredAccel, transform.position);
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
}
