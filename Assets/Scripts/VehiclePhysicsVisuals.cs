using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehiclePhysics
{
    public class VehiclePhysicsVisuals : MonoBehaviour
    {
        [Header("Direction Wheel")]
        [SerializeField] private Transform _directionWheel;
        [SerializeField] private bool _directionWheelInverseRot = true;
        [SerializeField] private float _directionWheelMultiplier = 1f;

        private VehiclePhysicsController _controller;

        private void Awake()
        {
            _controller = GetComponent<VehiclePhysicsController>();
        }

        private void Update()
        {
            if (_directionWheel)
                ApplyDirectionWheel();
        }

        private void ApplyDirectionWheel()
        {
            _directionWheel.localEulerAngles = new(_directionWheel.localEulerAngles.x,
                _directionWheel.localEulerAngles.y, (_directionWheelInverseRot ? _controller.SteeringAngle * -1 : _controller.SteeringAngle) * _directionWheelMultiplier);
        }
    }

}