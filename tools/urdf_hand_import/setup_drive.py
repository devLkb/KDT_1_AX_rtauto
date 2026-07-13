# -*- coding: utf-8 -*-
"""임포트된 로봇 프리팹에 '구동 준비' 일괄 적용 (범용, 멱등 — 재실행 안전).

적용 내용과 이유 (SVH 디버깅에서 확립, WORKLOG §6~12·§15 참고):
  1. 루트의 URDF-Importer Controller 컴포넌트 제거
     — Play 시 JointControl을 몰래 추가해 xDrive.target을 덮어쓰는 함정
  2. 전 revolute 관절 드라이브 게인 설정 (기본 10000/200/100000)
     — 임포트 직후 stiffness=0이라 모터 꺼진 상태.
     ⚠️ forceLimit을 URDF effort로 두면 stiffness 10000 기준 오차 0.04°에도 토크 포화
       → 뱅뱅 진동 (§18 손목 사태 패턴). 그래서 사실상 무제한(100000)으로 올림
       (HW 토크상한 재현 포기 트레이드오프).
  3. 전 바디 useGravity=false + 루트 immovable=true
     — 디지털 트윈은 명령 포즈 유지가 목적. 안 하면 Play 시 낙하+처짐
  4. 자기충돌 무시 + 초기 포즈 동기화 컴포넌트 부착
     — SvhSelfCollisionIgnore/SvhInitialPoseSync (이름만 Svh, 로직은 범용:
       자식 전체 순회라 어떤 로봇 루트에 붙여도 동작)

사용 예:
  python setup_drive.py dg5f_right dg5f_left            # Assets/Robots/Prefabs/<이름>.prefab
  python setup_drive.py dg5f_right --stiffness 5000 --damping 100
"""
import argparse
import sys
from pathlib import Path

import import_hand  # DEFAULT_CLI, unity_exec 재사용

SNIPPET = """
string[] prefabs = new string[] { @PREFABS@ };
float stiffness = @STIFF@f; float damping = @DAMP@f; float forceLimit = @FORCE@f;
string[] comps = new string[] { @COMPS@ };
System.Text.StringBuilder sb = new System.Text.StringBuilder();
foreach (string path in prefabs) {
  if (UnityEditor.AssetDatabase.LoadAssetAtPath<UnityEngine.GameObject>(path) == null) { sb.Append(path + ":NOT_FOUND | "); continue; }
  UnityEngine.GameObject root = UnityEditor.PrefabUtility.LoadPrefabContents(path);
  int removed = 0;
  foreach (UnityEngine.Component c in root.GetComponents<UnityEngine.Component>()) {
    if (c != null && c.GetType().FullName.Contains("UrdfImporter.Control")) { UnityEngine.Object.DestroyImmediate(c); removed++; }
  }
  UnityEngine.ArticulationBody[] abs = root.GetComponentsInChildren<UnityEngine.ArticulationBody>();
  int rev = 0;
  foreach (UnityEngine.ArticulationBody b in abs) {
    b.useGravity = false;
    if (b.jointType == UnityEngine.ArticulationJointType.RevoluteJoint) {
      UnityEngine.ArticulationDrive d = b.xDrive;
      d.stiffness = stiffness; d.damping = damping; d.forceLimit = forceLimit;
      b.xDrive = d; rev++;
    }
  }
  abs[0].immovable = true;
  int added = 0;
  foreach (string cn in comps) {
    System.Type t = System.Type.GetType(cn + ",Assembly-CSharp");
    if (t == null) { sb.Append("(" + cn + ":TYPE_NOT_FOUND)"); continue; }
    if (root.GetComponent(t) == null) { root.AddComponent(t); added++; }
  }
  UnityEditor.PrefabUtility.SaveAsPrefabAsset(root, path);
  UnityEditor.PrefabUtility.UnloadPrefabContents(root);
  sb.Append(path + ":ctrlRemoved=" + removed + ",rev=" + rev + ",compAdded=" + added + " | ");
}
return sb.ToString();
"""


def setup(prefab_names, cli, stiffness, damping, force_limit, components):
    paths = []
    for n in prefab_names:
        p = n if n.startswith("Assets/") else f"Assets/Robots/Prefabs/{n}.prefab"
        paths.append('"' + p + '"')
    code = (SNIPPET
            .replace("@PREFABS@", ", ".join(paths))
            .replace("@STIFF@", str(stiffness))
            .replace("@DAMP@", str(damping))
            .replace("@FORCE@", str(force_limit))
            .replace("@COMPS@", ", ".join('"' + c + '"' for c in components)))
    out = import_hand.unity_exec(Path(cli), code)
    print(out.replace(" | ", "\n"))
    return "NOT_FOUND" not in out and "TYPE_NOT_FOUND" not in out


def main():
    ap = argparse.ArgumentParser(description="로봇 프리팹 구동 준비 일괄 적용")
    ap.add_argument("prefab", nargs="+", help="프리팹 이름(Assets/Robots/Prefabs/ 기준) 또는 Assets/ 전체경로")
    ap.add_argument("--cli", type=Path, default=import_hand.DEFAULT_CLI)
    ap.add_argument("--stiffness", type=float, default=10000)
    ap.add_argument("--damping", type=float, default=200)
    ap.add_argument("--force-limit", type=float, default=100000)
    ap.add_argument("--components", nargs="*",
                    default=["SvhSelfCollisionIgnore", "SvhInitialPoseSync"],
                    help="루트에 부착할 컴포넌트 클래스명")
    args = ap.parse_args()
    ok = setup(args.prefab, args.cli, args.stiffness, args.damping, args.force_limit, args.components)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
