using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehiclePhysics
{
    public class VehiclePhysicsDamage : MonoBehaviour
    {
        [SerializeField] private float _maxMoveDelta = 1.0f; // maximum distance one vertice moves per explosion (in meters)
        [SerializeField] private float _maxCollisionStrength = 50.0f;
        [SerializeField] private float _yForceDamp = 0.1f; // 0.0 - 1.0
        [SerializeField] private float _demolutionRange = 0.5f;
        [SerializeField] private float _impactDirManipulator = 0.0f;
        [SerializeField] private Transform _meshParent;

        [SerializeField] private MeshFilter[] _meshfilters;
        private float _sqrDemRange;

        //Save Vertex Data
        private struct permaVertsColl
        {
            public Vector3[] permaVerts;
        }
        private permaVertsColl[] originalMeshData;
        int i;

        private void Start()
        {
            if(_meshfilters.Length <= 0 & _meshParent)
                _meshfilters = _meshParent.GetComponentsInChildren<MeshFilter>();

            _sqrDemRange = _demolutionRange * _demolutionRange;

            LoadOriginalMeshData();

        }

        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.R)) Repair();
        }

        void LoadOriginalMeshData()
        {
            originalMeshData = new permaVertsColl[_meshfilters.Length];
            for (i = 0; i < _meshfilters.Length; i++)
            {
                originalMeshData[i].permaVerts = _meshfilters[i].mesh.vertices;
            }
        }

        void Repair()
        {
            for (int i = 0; i < _meshfilters.Length; i++)
            {
                _meshfilters[i].mesh.vertices = originalMeshData[i].permaVerts;
                _meshfilters[i].mesh.RecalculateNormals();
                _meshfilters[i].mesh.RecalculateBounds();
            }
        }

        private void OnCollisionEnter(Collision collision)
        {
            Vector3 colRelVel = collision.relativeVelocity;
            colRelVel.y *= _yForceDamp;

            Vector3 colPointToMe = transform.position - collision.contacts[0].point;

            // Dot = angle to collision point, frontal = highest damage, strip = lowest damage
            float colStrength = colRelVel.magnitude * Vector3.Dot(collision.contacts[0].normal, colPointToMe.normalized);

            OnMeshForce(collision.contacts[0].point, Mathf.Clamp01(colStrength / _maxCollisionStrength));

        }

        // if called by SendMessage(), we only have 1 param
        public void OnMeshForce(Vector4 originPosAndForce)
        {
            OnMeshForce((Vector3)originPosAndForce, originPosAndForce.w);
        }

        private void OnMeshForce(Vector3 originPos, float force)
        {
            // force should be between 0.0 and 1.0
            force = Mathf.Clamp01(force);

            for (int j = 0; j < _meshfilters.Length; ++j)
            {
                Vector3[] verts = _meshfilters[j].mesh.vertices;

                for (int i = 0; i < verts.Length; ++i)
                {
                    Vector3 scaledVert = Vector3.Scale(verts[i], transform.localScale);
                    Vector3 vertWorldPos = _meshfilters[j].transform.position + (_meshfilters[j].transform.rotation * scaledVert);
                    Vector3 originToMeDir = vertWorldPos - originPos;
                    Vector3 flatVertToCenterDir = transform.position - vertWorldPos;
                    flatVertToCenterDir.y = 0.0f;

                    // 0.5 - 1 => 45° to 0°  / current vertice is nearer to exploPos than center of bounds
                    if (originToMeDir.sqrMagnitude < _sqrDemRange) //dot > 0.8f )
                    {
                        float dist = Mathf.Clamp01(originToMeDir.sqrMagnitude / _sqrDemRange);
                        float moveDelta = force * (1.0f - dist) * _maxMoveDelta;

                        Vector3 moveDir = Vector3.Slerp(originToMeDir, flatVertToCenterDir, _impactDirManipulator).normalized * moveDelta;

                        verts[i] += Quaternion.Inverse(transform.rotation) * moveDir;
                    }

                }

                _meshfilters[j].mesh.vertices = verts;
                _meshfilters[j].mesh.RecalculateBounds();
            }
        }
    }
}