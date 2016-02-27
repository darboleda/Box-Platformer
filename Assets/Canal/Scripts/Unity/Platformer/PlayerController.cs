using UnityEngine;
using System.Collections;

namespace Canal.Unity.Platformer
{
    [RequireComponent(typeof(PlatformerMotor))]
    public class PlayerController : MonoBehaviour
    {
        public float WalkSpeed = 7.0f;
        public float JumpSpeed = 20;
        public float BaseAcceleration = 1.0f;
        public float BaseDeceleration = 4.0f;
        public float MaxFallSpeed = 10f;
        public float StopDistance = 0.01f;
        public float BaseGravity = 9.8f;

        public bool ShouldPause = false;

        public LayerMask solidLayers;
        public LayerMask platformLayers;

        public Animator Animator;

        public Transform CeilingSensors;
        public Transform FloorSensors;
        public Transform LeftSensors;
        public Transform RightSensors;

        private PlatformerMotor motor;
        private bool onGround;

        public int BaseAirActions = 1;
        private int airActions = 0;

        public void Awake()
        {
            GetComponent<Rigidbody>().constraints = RigidbodyConstraints.FreezeRotation;
            motor = GetComponent<PlatformerMotor>();

            motor.UpdatedX += motor_UpdatedX;
            motor.UpdatedY += motor_UpdatedY;
        }

        void motor_UpdatedX(Vector3 oldWorldPosition, Vector3 localDelta, Vector3 velocity)
        {
            if (!this.enabled) return;

            HandleWallCollisions(oldWorldPosition.x, localDelta.x, velocity.x, 0.5f);
        }

        void motor_UpdatedY(Vector3 oldPos, Vector3 delta, Vector3 velocity)
        {
            if (!this.enabled) return;

            HandleCeilingCollisions(oldPos.y, delta.y, velocity.y, 1);
            HandleFloorCollisions(oldPos.y, delta.y, velocity.y, 1);
        }

        private struct LandableArgs
        {
            public RaycastHit hit;
            public bool wasOnGround;
            public Vector3 sensorPosition;
            public Vector3 hitPosition;
            public Vector3 hitVector;
            public float vSpeed;
            public float deltaY;
        }

        private bool IsLandable(LandableArgs args)
        {
            return args.vSpeed <= 0 && (args.wasOnGround || args.hitVector.y >= 0);

            /*
            ((localDirection.y >= 0
                          && sensorLocal.y - (newY - oldY) >= motor.transform.InverseTransformPoint(hit.point).y)
                         || onGround)
                        && vSpeed <= 0
                        */
        }

        private void HandleFloorCollisions(float oldY, float deltaY, float vSpeed, float senseDistance)
        {
            float distance = senseDistance;
            Vector3 worldDown = Vector3.down * distance;
            Vector3 worldDownDirection = Vector3.down;
            Vector3 bestPoint = Vector3.up * float.NegativeInfinity;
            bool wasOnGround = onGround;
            
            foreach (Transform sensor in FloorSensors)
            {
                if (!sensor.gameObject.activeInHierarchy) continue;
                Ray ray = new Ray(sensor.position - worldDown, worldDownDirection);

                RaycastHit hit;
                if (Physics.Raycast(ray, out hit, distance * 1.25f, solidLayers | platformLayers))
                {

                    Vector3 sensorPosition = sensor.position;
                    Vector3 direction = hit.point - sensorPosition;

                    LandableArgs args = new LandableArgs();
                    args.sensorPosition = sensorPosition;
                    args.hitPosition = hit.point;
                    args.hitVector = direction;
                    args.vSpeed = vSpeed;
                    args.wasOnGround = wasOnGround;
                    args.hit = hit;
                    args.deltaY = deltaY;

                    if (IsLandable(args)
                        && direction.y > bestPoint.y)
                    {
                        Debug.DrawLine(ray.origin, hit.point, Color.red, 0, false);
                        bestPoint = direction;
                        onGround = true;
                    }
                    else
                    {
                        Debug.DrawLine(ray.origin, hit.point, Color.white, 0, false);
                    }
                }
                else
                {
                    Debug.DrawRay(sensor.position, worldDownDirection * senseDistance, Color.blue, 0, false);
                }
            }
            if (!float.IsInfinity(bestPoint.y))
            {
                motor.Transform.position += bestPoint;
                Vector3 vel = motor.Velocity;
                vel.y = 0;
                motor.Velocity = vel;
            }
            else
            {
                onGround = false;
            }

            if (vSpeed <= 0 && !onGround && wasOnGround)
            {
                Animator.SetTrigger("Fall");
                if (ShouldPause)
                {
                    Debug.Break();
                }
            }

            if (onGround)
            {
                airActions = BaseAirActions;
            }
        }

        private void HandleCeilingCollisions(float oldY, float deltaY, float vSpeed, float senseDistance)
        {
            Transform sensors = CeilingSensors;

            if (sensors == null) return;

            float distance = senseDistance;
            Vector3 direction = Vector3.up;
            float bestDelta = float.PositiveInfinity;
            foreach (Transform sensor in sensors)
            {
                Ray ray = new Ray(sensor.position - direction * distance, direction);

                RaycastHit hit;
                if (Physics.Raycast(ray, out hit, distance, solidLayers))
                {
                    Debug.DrawLine(ray.origin, hit.point, Color.red, 0, false);
                    float delta = sensor.position.y - hit.point.y;
                    if (delta < bestDelta)
                    {
                        bestDelta = delta;
                        Vector3 vel = motor.Velocity;
                        vel.y = 0;
                        motor.Velocity = vel;
                    }
                }
                else
                {
                    Debug.DrawRay(ray.origin, ray.direction * senseDistance, Color.white, 0, false);
                }
            }

            if (!float.IsInfinity(bestDelta))
            {
                motor.Transform.position += Vector3.down * bestDelta;
            }
        }

        private void HandleWallCollisions(float oldX, float deltaX, float hSpeed, float senseDistance)
        {
            HandleWallCollision(oldX, 1, hSpeed, senseDistance);
            HandleWallCollision(oldX, -1, hSpeed, senseDistance);
        }

        private void HandleWallCollision(float oldX, float deltaX, float hSpeed, float senseDistance)
        {
            Transform sensors = (deltaX > 0 ? RightSensors :
                                 deltaX < 0 ? LeftSensors : null);

            if (sensors == null) return;

            float distance = senseDistance;
            Vector3 direction = (deltaX > 0 ? Vector3.right : Vector3.left);
            float bestDelta = 0;
            foreach (Transform sensor in sensors)
            {
                Ray ray = new Ray(sensor.position - direction * distance, direction);

                RaycastHit hit;
                if (Physics.Raycast(ray, out hit, distance, solidLayers))
                {
                    float delta = sensor.position.x - hit.point.x;
                    if (deltaX > 0 && delta > bestDelta)
                    {
                        bestDelta = delta;
                    }
                    else if (deltaX < 0 && delta < bestDelta)
                    {
                        bestDelta = delta;
                    }
                    Debug.DrawLine(ray.origin, hit.point, Color.red, 0, false);
                    Vector3 vel = motor.Velocity;
                    vel.x = 0;
                    motor.Velocity = vel;
                }
                else
                {
                    Debug.DrawRay(ray.origin, ray.direction * senseDistance, Color.white, 0, false);
                }
            }

            motor.Transform.position += Vector3.left * bestDelta;
        }

        public void Update()
        {
            if (Input.GetButtonDown("Jump") && (onGround || airActions > 0))
            {
                Vector3 vel = motor.Velocity;
                vel.y = JumpSpeed;
                motor.Velocity = vel;

                if (onGround)
                {
                    Animator.SetTrigger("Jump");
                }
                else
                {
                    --airActions;
                    Animator.SetTrigger("DoubleJump");
                }
            }

            if (Input.GetButtonUp("Jump") && motor.Velocity.y > 0)
            {
                Vector3 vel = motor.Velocity;
                vel.y = vel.y / 2;
                motor.Velocity = vel;
            }
            
            Animator.SetBool("Crouching", Input.GetAxisRaw("Vertical") < 0);

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

            float degrees = Mathf.Sign(motor.Velocity.x) * 90;
            Animator.transform.localRotation = Quaternion.Euler(new Vector3(0, degrees, 0));
            Animator.SetBool("OnGround", onGround);
            Animator.SetFloat("Speed", Mathf.Abs(motor.Velocity.x) / WalkSpeed);

            Animator.transform.position = Vector3.Lerp(Camera.main.transform.position, motor.Transform.position, 0.9f);
            Animator.transform.localScale = Vector3.one * 0.9f;
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