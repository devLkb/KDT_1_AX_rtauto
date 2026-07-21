using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>Tracks contact between one fingertip and the configured training ball.</summary>
    public sealed class GraspContactSensor : MonoBehaviour
    {
        [Range(0, Dg5fGraspSpec.FingerCount - 1)]
        public int fingerIndex;
        public Rigidbody targetBall;

        readonly HashSet<Collider> _contacts = new HashSet<Collider>();

        public bool IsTouching => _contacts.Count > 0;

        public void ResetContacts()
        {
            _contacts.Clear();
        }

        bool IsTarget(Collider other)
        {
            if (targetBall == null || other == null) return false;
            return other.attachedRigidbody == targetBall
                || other.transform.IsChildOf(targetBall.transform);
        }

        void OnCollisionEnter(Collision collision)
        {
            if (IsTarget(collision.collider)) _contacts.Add(collision.collider);
        }

        void OnCollisionStay(Collision collision)
        {
            if (IsTarget(collision.collider)) _contacts.Add(collision.collider);
        }

        void OnCollisionExit(Collision collision)
        {
            if (collision.collider != null) _contacts.Remove(collision.collider);
        }

        void OnDisable()
        {
            _contacts.Clear();
        }
    }
}
