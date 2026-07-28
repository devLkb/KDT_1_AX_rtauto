using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// Tracks contact between one grasp contact point (fingertip 0..4, or the palm
    /// at index <see cref="Dg5fGraspLiftSpec.PalmContactIndex"/>) and the block.
    /// </summary>
    public sealed class GraspLiftObjectContactSensor : MonoBehaviour
    {
        [Range(0, Dg5fGraspLiftSpec.ContactPointCount - 1)]
        public int contactIndex;
        public Rigidbody targetObject;

        readonly HashSet<Collider> _contacts = new HashSet<Collider>();

        public bool IsTouching => _contacts.Count > 0;

        /// Magnitude of the most recent collision impulse against the block. Used
        /// only for diagnostics/stats — the grasp contract is geometric, so a weak
        /// but well-placed contact still counts.
        public float LastImpulse { get; private set; }

        public void ResetContacts()
        {
            _contacts.Clear();
            LastImpulse = 0f;
        }

        bool IsTarget(Collider other)
        {
            if (targetObject == null || other == null) return false;
            return other.attachedRigidbody == targetObject
                || other.transform.IsChildOf(targetObject.transform);
        }

        void Register(Collision collision)
        {
            if (collision == null || !IsTarget(collision.collider)) return;
            _contacts.Add(collision.collider);
            LastImpulse = collision.impulse.magnitude;
        }

        void OnCollisionEnter(Collision collision)
        {
            Register(collision);
        }

        void OnCollisionStay(Collision collision)
        {
            Register(collision);
        }

        void OnCollisionExit(Collision collision)
        {
            if (collision != null && collision.collider != null)
                _contacts.Remove(collision.collider);
            if (_contacts.Count == 0) LastImpulse = 0f;
        }

        void OnDisable()
        {
            ResetContacts();
        }
    }
}
