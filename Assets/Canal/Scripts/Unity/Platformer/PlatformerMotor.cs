using UnityEngine;
using System.Collections;

namespace Canal.Unity.Platformer
{
    [RequireComponent(typeof(Rigidbody))]
    public class PlatformerMotor : MonoBehaviour
    {
        private Transform cachedTransform;
        public Transform Transform
        {
            get
            {
                if (cachedTransform == null)
                {
                    cachedTransform = transform;
                }
                return cachedTransform;
            }
        }

        public Vector3 Velocity;

        public delegate void PositionUpdatedHandler(Vector3 oldWorldPosition, Vector3 localDelta, Vector3 velocity);

        public event PositionUpdatedHandler UpdatedX = (x, y, z) => { };
        public event PositionUpdatedHandler UpdatedY = (x, y, z) => { };
        public event PositionUpdatedHandler UpdatedZ = (x, y, z) => { };

        protected virtual void FixedUpdate()
        {
            float dt = Time.deltaTime;
            Vector3 velocity = Velocity;
            velocity = Transform.localRotation * velocity;

            Vector3 delta, currentPosition;

            currentPosition = Transform.position;
            delta = Vector3.right * velocity.x * dt;
            Transform.position += delta;
            UpdatedX(currentPosition, delta, Velocity);

            currentPosition = Transform.position;
            delta = Vector3.forward * velocity.z * dt;
            Transform.position += delta;
            UpdatedZ(currentPosition, delta, Velocity);

            currentPosition = Transform.position;
            delta = Vector3.up * velocity.y * dt;
            Transform.position += delta;
            UpdatedY(currentPosition, delta, Velocity);


            if (!GetComponent<Rigidbody>().isKinematic)
            {
                GetComponent<Rigidbody>().velocity = Vector3.zero;
            }
        }

        public Vector3 ApplyForce(Vector3 force, float dt)
        {
            Velocity += force * dt;
            return Velocity;
        }
    }
}