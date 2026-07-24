using System.Linq;
using NUnit.Framework;
using Unity.InferenceEngine;
using UnityEditor;

namespace KDT.GraspTraining.Tests
{
    public sealed class Dg5fGraspModelTests
    {
        const string ExistingModelPath =
            "Assets/MLAgents/Grasp/Models/DG5FGrasp-599887.onnx";

        [Test]
        public void Existing599887OnnxStillImportsAndLoads()
        {
            ModelAsset asset = AssetDatabase.LoadAssetAtPath<ModelAsset>(
                ExistingModelPath);
            Assert.That(asset, Is.Not.Null);

            Model model = null;
            Assert.DoesNotThrow(() => model = ModelLoader.Load(asset));
            Assert.That(model, Is.Not.Null);
            Assert.That(model.inputs.Select(input => input.name), Does.Contain("obs_0"));
            Assert.That(
                model.outputs.Select(output => output.name),
                Does.Contain("continuous_actions"));
        }
    }
}
