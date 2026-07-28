using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspLiftTraining
{
    /// <summary>
    /// Non-terminal reporter for contact between a hand collider and the panel.
    /// The agent polls all instances and ORs their state, so the component works
    /// when placed on a link or on the collider GameObject that receives collision
    /// callbacks from the URDF hierarchy.
    /// </summary>
    public sealed class GraspLiftHandSurfaceSensor : MonoBehaviour
    {
        public Collider surface;

        readonly HashSet<Collider> _contacts = new HashSet<Collider>();

        public bool IsTouching => _contacts.Count > 0;

        public void ResetContacts()
        {
            _contacts.Clear();
        }

        bool IsSurface(Collider other)
        {
            return surface != null && other != null
                && (other == surface || other.transform.IsChildOf(surface.transform));
        }

        void Register(Collider other)
        {
            if (IsSurface(other)) _contacts.Add(other);
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
            ResetContacts();
        }
    }
}
