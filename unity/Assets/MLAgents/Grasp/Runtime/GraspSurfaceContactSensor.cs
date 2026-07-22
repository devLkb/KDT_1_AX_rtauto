using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>Reports contact between one moving robot collider and the panel.</summary>
    public sealed class GraspSurfaceContactSensor : MonoBehaviour
    {
        public Dg5fGraspAgent agent;
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

        void OnCollisionEnter(Collision collision)
        {
            Register(collision.collider);
        }

        void OnCollisionStay(Collision collision)
        {
            Register(collision.collider);
        }

        void OnCollisionExit(Collision collision)
        {
            if (collision.collider != null) _contacts.Remove(collision.collider);
        }

        void Register(Collider other)
        {
            if (!IsUnsafe(other)) return;
            _contacts.Add(other);
            if (agent != null) agent.NotifyUnsafeSurfaceContact(unsafeSurface);
        }

        void OnDisable()
        {
            _contacts.Clear();
        }
    }
}
