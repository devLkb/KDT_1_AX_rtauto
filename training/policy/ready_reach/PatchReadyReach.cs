using System;
using System.IO;
using System.Linq;
using Mono.Cecil;
using Mono.Cecil.Cil;

/// <summary>
/// Adds the curriculum layer to the packaged ReadyReach assembly without
/// changing its 37-observation / 6-action neural-network contract.
/// </summary>
static class PatchReadyReach
{
    static MethodDefinition FindMethod(TypeDefinition type, string name)
    {
        return type.Methods.Single(method => method.Name == name);
    }

    static void ForwardTo(
        MethodDefinition target,
        MethodReference replacement,
        ModuleDefinition module)
    {
        target.Body = new MethodBody(target);
        ILProcessor il = target.Body.GetILProcessor();
        foreach (ParameterDefinition parameter in target.Parameters)
            il.Append(il.Create(OpCodes.Ldarg, parameter));
        il.Append(il.Create(OpCodes.Call, module.ImportReference(replacement)));
        il.Append(il.Create(OpCodes.Ret));
    }

    static int Main(string[] args)
    {
        if (args.Length != 3)
        {
            Console.Error.WriteLine(
                "usage: PatchReadyReach input.dll curriculum.dll output.dll");
            return 2;
        }

        var resolver = new DefaultAssemblyResolver();
        resolver.AddSearchDirectory(Path.GetDirectoryName(args[0]));
        resolver.AddSearchDirectory(Path.GetDirectoryName(args[1]));
        var reader = new ReaderParameters { AssemblyResolver = resolver };

        using (AssemblyDefinition assembly =
            AssemblyDefinition.ReadAssembly(args[0], reader))
        using (AssemblyDefinition curriculumAssembly =
            AssemblyDefinition.ReadAssembly(args[1], reader))
        {
            ModuleDefinition module = assembly.MainModule;
            TypeDefinition reachSpec = module.Types.Single(type =>
                type.FullName == "KDT.ReachTraining.Dg5fReachSpec");
            TypeDefinition agent = module.Types.Single(type =>
                type.FullName ==
                "KDT.ReachTraining.Dg5fGraspPointReachAgent");
            TypeDefinition curriculum =
                curriculumAssembly.MainModule.Types.Single(type =>
                    type.FullName ==
                    "KDT.ReachTraining.ReadyReachCurriculum");

            ForwardTo(
                FindMethod(reachSpec, "MeetsLockState"),
                FindMethod(curriculum, "MeetsLockState"),
                module);
            ForwardTo(
                FindMethod(reachSpec, "LockHoldProgress"),
                FindMethod(curriculum, "LockHoldProgress"),
                module);
            ForwardTo(
                FindMethod(reachSpec, "HasCompletedLockHold"),
                FindMethod(curriculum, "HasCompletedLockHold"),
                module);

            MethodDefinition episodeBegin =
                FindMethod(agent, "OnEpisodeBegin");
            ILProcessor il = episodeBegin.Body.GetILProcessor();
            il.InsertBefore(
                episodeBegin.Body.Instructions[0],
                il.Create(
                    OpCodes.Call,
                    module.ImportReference(
                        FindMethod(curriculum, "RefreshStage"))));

            assembly.Write(args[2]);
        }
        return 0;
    }
}
