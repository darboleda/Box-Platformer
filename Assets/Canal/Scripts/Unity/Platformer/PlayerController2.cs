using UnityEngine;
using System.Collections;

namespace Canal.Unity.Platformer
{
    public class PlayerController2 : MonoBehaviour
    {
        public float WalkSpeed = 7.0f;
        public float JumpSpeed = 20;
        public float BaseAcceleration = 1.0f;
        public float BaseDeceleration = 4.0f;
        public float MaxFallSpeed = 10f;
        public float StopDistance = 0.01f;
        public float BaseGravity = 9.8f;
        public Transform LeftFloorSensor, RightFloorSensor;

        private bool onGround;

        public int BaseAirActions = 1;
        private int airActions = 0;

        private PlatformerMotor motor;
        public void Awake()
        {
            GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeRotation;
            motor = GetComponent<PlatformerMotor>();
            motor.UpdatedY += motor_UpdatedY;
        }

        void motor_UpdatedY(Vector3 newPosition, Vector3 oldPosition, Vector3 velocity)
        {
            if (!this.enabled) return;

            HandleFloorCollisions(newPosition.y, oldPosition.y, velocity.y);
        }

        private void HandleFloorCollisions(float newY, float oldY, float vSpeed)
        {
            Vector3 worldDown = motor.Transform.TransformDirection(Vector3.down);
            Debug.DrawLine(LeftFloorSensor.position, LeftFloorSensor.position + worldDown * 20, Color.white);
            Debug.DrawLine(RightFloorSensor.position, RightFloorSensor.position + worldDown * 20, Color.white);

            RaycastHit leftHit, rightHit;
            float bestY = float.NegativeInfinity;
            bool left = false, right = false;
            if (CheckSensor(LeftFloorSensor, worldDown, 1.5f, out leftHit))
            {
                Debug.DrawLine(LeftFloorSensor.position - worldDown, leftHit.point, Color.yellow);
                bestY = leftHit.point.y;
                left = true;
            }
            else
            {
                Debug.DrawLine(LeftFloorSensor.position - worldDown, LeftFloorSensor.position + worldDown * 20, Color.white);
            }

            if (CheckSensor(RightFloorSensor, worldDown, 1.5f, out rightHit))
            {
                Debug.DrawLine(RightFloorSensor.position - worldDown, rightHit.point, Color.yellow);
                bestY = Mathf.Max(bestY, rightHit.point.y);
                right = true;
            }
            else
            {
                Debug.DrawLine(RightFloorSensor.position - worldDown, RightFloorSensor.position + worldDown * 20, Color.white);
            }

            if (left && right)
            {
                if (leftHit.normal != rightHit.normal)
                {

                    RaycastHit hit;
                    Vector3 dir = rightHit.point - leftHit.point;
                    if (Physics.Raycast(leftHit.point - dir.normalized * 0.01f, dir.normalized, out hit, dir.magnitude) && (hit.point - rightHit.point).sqrMagnitude > 0.01f)
                    {
                        Debug.DrawLine(leftHit.point, rightHit.point, Color.red);
                        Vector3 position = motor.Transform.position;
                        Vector3 otherPosition = position;
                        otherPosition.y = Mathf.Max(leftHit.collider.bounds.max.y, rightHit.collider.bounds.max.y);
                        Debug.DrawLine(motor.Transform.position, otherPosition, Color.blue);
                        bestY = otherPosition.y;
                    }
                    else
                    {
                        Debug.DrawLine(leftHit.point, rightHit.point, Color.green);
                    }
                }
                else
                {
                    bestY = Mathf.Max(leftHit.point.y, rightHit.point.y);
                    Debug.DrawLine(leftHit.point, rightHit.point, Color.green);
                }
            }
            else if (left)
            {
                bestY = Mathf.Max(bestY, leftHit.collider.bounds.max.y);
            }
            else if (right)
            {
                bestY = Mathf.Max(bestY, rightHit.collider.bounds.max.y);
            }

            if (!float.IsInfinity(bestY) && (onGround || (bestY >= newY && bestY <= oldY)) && vSpeed <= 0)
            {
                Vector3 position = motor.Transform.position;
                position.y = bestY;
                motor.Transform.position = position;
                

                Vector3 vel = motor.Velocity;
                vel.y = 0;
                motor.Velocity = vel;

                onGround = true;
            }
            else
            {
                onGround = false;
            }
        }

        public bool CheckSensor(Transform sensor, Vector3 worldDirection, float distance, out RaycastHit hit)
        {

            if (Physics.Raycast(sensor.position - worldDirection, worldDirection.normalized, out hit, distance * worldDirection.magnitude + 1))
            {
                return true;
            }
            hit = new RaycastHit();
            return false;
        }

        public void Update()
        {
            if (Input.GetButtonDown("Jump") && (onGround || airActions > 0))
            {
                motor.Velocity.y = JumpSpeed;

                if (onGround)
                {
                    airActions = BaseAirActions;
                }
                else
                {
                    --airActions;
                }
            }

            float dt = Time.deltaTime;
            float x = Input.GetAxisRaw("Horizontal");
            Vector3 move = Vector3.zero;
            move.x = x;

            ApplyWalk(move, BaseAcceleration, BaseDeceleration, WalkSpeed, dt);
            if (!onGround)
            {
                float oldSpeed = motor.Velocity.y;
                Vector3 newSpeed = ApplyGravity(BaseGravity, MaxFallSpeed, dt);
            }
        }

        private Vector3 ApplyWalk(Vector3 moveDirection, float acceleration, float deceleration, float walkSpeed, float dt)
        {
            Vector3 currentVelocity = motor.Velocity;
            Vector3 walkVelocity = currentVelocity;
            float walkMagnitude = Mathf.Abs(walkVelocity.x);
            float magnitude = Mathf.Abs(moveDirection.x);
            if (walkMagnitude > magnitude * walkSpeed
                && (moveDirection.x == 0 || Mathf.Sign(walkVelocity.x) == Mathf.Sign(moveDirection.x)))
            {
                float targetDirection = Mathf.Sign(moveDirection.x - currentVelocity.x);
                float oldDirection = Mathf.Sign(currentVelocity.x);
                currentVelocity = motor.ApplyForce(Vector3.right * targetDirection * deceleration, dt);
                if (Mathf.Abs(currentVelocity.x) < StopDistance
                    || Mathf.Sign(currentVelocity.x) != oldDirection)
                {
                    currentVelocity.x = 0;
                }
                motor.Velocity = currentVelocity;
            }
            else
            {
                motor.ApplyForce(moveDirection * acceleration, dt);
            }

            return currentVelocity;
        }

        private Vector3 ApplyGravity(float gravity, float maxFallSpeed, float dt)
        {
            Vector3 currentVelocity = motor.ApplyForce(Vector3.down * gravity, dt);
            if (currentVelocity.y < -maxFallSpeed)
            {
                currentVelocity.y = -maxFallSpeed;
                motor.Velocity = currentVelocity;
            }

            return currentVelocity;
        }
    }
}