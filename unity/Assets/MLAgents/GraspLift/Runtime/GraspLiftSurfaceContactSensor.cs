using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// Reports contact between one moving arm collider and the panel. Hand links are
    /// deliberately NOT instrumented: the fingers have to work right at the table to
    /// grasp the block, so treating their contact as a safety failure would make the
    /// task impossible.
    /// </summary>
    public sealed class GraspLiftSurfaceContactSensor : MonoBehaviour
    {
        public Dg5fGraspLiftAgent agent;
        public Collider unsafeSurface;

        readonly HashSet<Collider> _contacts = new HashSet<Collider>();

        public bool HasUnsafeContact => _contacts.Count > 0;

        public void ResetContacts()
        {
            _contacts.Clear();
        }

        bool IsUnsafe(Collider other)
        {
            return unsafeSurface != null && other != null
                && (other == unsafeSurface
                    || other.transform.IsChildOf(unsafeSurface.transform));
        }

        void Register(Collider other)
        {
            if (!IsUnsafe(other)) return;
            _contacts.Add(other);
            if (agent != null) agent.NotifyUnsafeSurfaceContact(unsafeSurface);
        }

        void OnCollisionEnter(Collision collision)
        {
            if (collision != null) Register(collision.collider);
        }

        void OnCollisionStay(Collision collision)
        {
            if (collision != null) Register(collision.collider);
        }

        void OnCollisionExit(Collision collision)
        {
            if (collision != null && collision.collider != null)
                _contacts.Remove(collision.collider);
        }

        void OnDisable()
        {
            _contacts.Clear();
        }
    }
}
