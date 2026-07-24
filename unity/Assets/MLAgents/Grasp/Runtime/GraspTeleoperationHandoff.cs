using System;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Keeps all MediaPipe hand writers disabled until the reach policy has
    /// latched the arm, then transfers exclusive ownership of the hand joints.
    /// </summary>
    public sealed class GraspTeleoperationHandoff : MonoBehaviour
    {
        public Dg5fGraspAgent agent;
        public MonoBehaviour[] teleoperationDrivers = Array.Empty<MonoBehaviour>();

        public bool IsTeleoperationActive { get; private set; }

        void Awake()
        {
            if (agent == null) agent = GetComponent<Dg5fGraspAgent>();
            SetTeleoperationDrivers(false);
        }

        void FixedUpdate()
        {
            bool shouldBeActive = agent != null
                && agent.IsArmLocked
                && agent.IsExternalHandControl;
            if (shouldBeActive == IsTeleoperationActive) return;
            SetTeleoperationDrivers(shouldBeActive);
        }

        public void ReleaseForNextTarget()
        {
            SetTeleoperationDrivers(false);
            if (agent != null) agent.ReleaseArmLock();
        }

        void SetTeleoperationDrivers(bool enabled)
        {
            if (teleoperationDrivers != null)
            {
                foreach (MonoBehaviour driver in teleoperationDrivers)
                    if (driver != null) driver.enabled = enabled;
            }
            IsTeleoperationActive = enabled;
        }
    }
}
