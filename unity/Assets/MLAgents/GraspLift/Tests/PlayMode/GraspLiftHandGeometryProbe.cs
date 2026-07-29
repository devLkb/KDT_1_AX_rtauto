using System.Collections;
using System.Linq;
using System.Text;
using KDT.GraspLiftTraining;
using NUnit.Framework;
using Unity.MLAgents;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

namespace KDT.GraspLiftTraining.PlayModeTests
{
    /// <summary>
    /// Measurement harness, not a pass/fail contract. Closes the DG5F with nothing in
    /// the way and reports where the fingertips actually end up, so the block size and
    /// the palm-local grasp point are chosen from simulated geometry instead of from
    /// URDF arithmetic. Run it and read the log:
    ///
    ///   -testFilter "KDT.GraspLiftTraining.PlayModeTests.GraspLiftHandGeometryProbe"
    /// </summary>
    public sealed class GraspLiftHandGeometryProbe
    {
        [UnityTest]
        public IEnumerator ReportClosedHandGeometry()
        {
            SceneManager.LoadScene("DG5F_GraspLiftTraining");
            yield return null;
            for (int i = 0; i < 8; i++) yield return new WaitForFixedUpdate();

            var agent = Object.FindObjectsByType<Dg5fGraspLiftAgent>(FindObjectsSortMode.None)
                .OrderBy(a => a.transform.root.name)
                .First();
            agent.GetComponent<DecisionRequester>().enabled = false;
            agent.enabled = false;

            // Get the block out of the way so the fingers close freely.
            agent.graspObject.isKinematic = true;
            agent.graspObject.position += Vector3.up * 5f;
            Physics.SyncTransforms();
            yield return new WaitForFixedUpdate();

            var bodies = agent.GetComponentsInChildren<ArticulationBody>(true);
            var handJoints = new ArticulationBody[Dg5fGraspLiftSpec.HandJointCount];
            for (int finger = 1; finger <= Dg5fGraspLiftSpec.FingerCount; finger++)
                for (int joint = 1; joint <= 4; joint++)
                {
                    string suffix = $"_dg_{finger}_{joint}";
                    handJoints[(finger - 1) * 4 + joint - 1] =
                        bodies.First(body => body.name.EndsWith(suffix));
                }
            var openDeg = handJoints.Select(joint => joint.xDrive.target).ToArray();

            Transform palm = agent.palm;
            Transform graspPoint = agent.graspPoint;
            Transform[] tips = agent.fingerTips;

            var report = new StringBuilder();
            report.AppendLine("[HandGeometryProbe] palm-local fingertip positions");
            report.AppendLine($"  configured GraspPoint local = {Dg5fGraspLiftSpec.FullHandGraspPointLocalPosition}");

            foreach (float closure in new[] { 0f, 0.5f, 0.75f, 1f })
            {
                for (int step = 0; step < 60; step++)
                {
                    for (int i = 0; i < handJoints.Length; i++)
                    {
                        var drive = handJoints[i].xDrive;
                        drive.target = Mathf.Clamp(
                            Mathf.Lerp(openDeg[i], Dg5fGraspLiftSpec.LeftFistDeg[i], closure),
                            drive.lowerLimit,
                            drive.upperLimit);
                        handJoints[i].xDrive = drive;
                    }
                    yield return new WaitForFixedUpdate();
                }

                Vector3 centroid = Vector3.zero;
                report.AppendLine($"  --- closure {closure:F2}");
                for (int i = 0; i < tips.Length; i++)
                {
                    Vector3 local = palm.InverseTransformPoint(tips[i].position);
                    centroid += local;
                    report.AppendLine($"      tip{i + 1} palmLocal = {local:F4}");
                }
                centroid /= tips.Length;
                report.AppendLine($"      centroid palmLocal = {centroid:F4}");
                report.AppendLine(
                    $"      centroid -> configured GraspPoint offset = "
                    + $"{(centroid - Dg5fGraspLiftSpec.FullHandGraspPointLocalPosition).magnitude:F4} m");
                report.AppendLine(
                    $"      graspPoint palmLocal = {palm.InverseTransformPoint(graspPoint.position):F4}");

                // Widest opposed pair: the usable aperture of the closed hand.
                float widest = 0f;
                string widestPair = "";
                for (int a = 0; a < tips.Length; a++)
                    for (int b = a + 1; b < tips.Length; b++)
                    {
                        float d = Vector3.Distance(tips[a].position, tips[b].position);
                        if (d > widest) { widest = d; widestPair = $"tip{a + 1}-tip{b + 1}"; }
                    }
                report.AppendLine($"      widest tip pair = {widestPair} at {widest:F4} m");

                float thumbToIndex = Vector3.Distance(tips[0].position, tips[1].position);
                float thumbToMiddle = Vector3.Distance(tips[0].position, tips[2].position);
                report.AppendLine(
                    $"      thumb-index = {thumbToIndex:F4} m, thumb-middle = {thumbToMiddle:F4} m");
            }

            Debug.Log(report.ToString());
            Assert.Pass("geometry reported to the log");
        }
    }
}
