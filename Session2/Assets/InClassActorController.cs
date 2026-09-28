// Setup: Attach to Actor (requires Rigidbody). Drag Target into the Inspector.

using UnityEngine;

namespace CS6983.Week2ICA
{
    [RequireComponent(typeof(Rigidbody))]
    public class InClassActorController : MonoBehaviour
    {
        [SerializeField] private Transform target;
        [SerializeField] private float moveStrength = 20f;

        private Rigidbody _body;

        private void Awake()
        {
            _body = GetComponent<Rigidbody>();
        }

        private void FixedUpdate()
        {
            if (target == null || _body == null)
                return;

            // Direction to target on the ground plane.
            Vector3 toTarget = target.position - transform.position;
            // Flatten the direction to the target.
            toTarget.y = 0f;

            if (toTarget.sqrMagnitude < 0.0001f)
                return;

            // TODO: apply force toward the target.
            Vector3 direction = toTarget.normalized;
            _body.AddForce(direction * moveStrength);
        }
    }
}
