// Setup: Collider with Is Trigger = true. Payload must use tag "Payload".

using System;
using UnityEngine;

namespace CS6983.Week2ICA
{
    [RequireComponent(typeof(Collider))]
    public class InClassGoalZone : MonoBehaviour
    {
        [SerializeField] private string payloadTag = "Payload";

        public event Action<float> PayloadSucceeded;

        private float _runStartTime;
        private bool _succeeded;

        public bool HasSucceeded => _succeeded;
        public float TimeToSuccess { get; private set; }

        private void Awake()
        {
            Collider col = GetComponent<Collider>();
            if (col != null && !col.isTrigger)
                Debug.LogWarning($"{name}: GoalZone collider should have Is Trigger enabled.", this);
        }

        private void OnEnable()
        {
            ResetRun();
        }

        private void OnDisable()
        {
            return;
        }

        public void ResetRun()
        {
            _succeeded = false;
            TimeToSuccess = 0f;
            _runStartTime = Time.time;
        }

        private void OnTriggerEnter(Collider other)
        {
            // 'other' = the collider that just entered this trigger.
            if (_succeeded)
                return;

            if (!IsPayload(other))
                return;

            // TODO: Mark success and fire the PayloadSucceeded event with the time to success.
            _succeeded = true;
            TimeToSuccess = Time.time - _runStartTime;
            PayloadSucceeded?.Invoke(TimeToSuccess);
            Debug.Log($"Payload succeeded in {TimeToSuccess:F2}s!", this);
        }

        private bool IsPayload(Collider other)
        {
            // TODO: Return true if this is the Payload using the payloadTag.
            // return false if it is not.
            return other.CompareTag(payloadTag);
        }
    }
}
