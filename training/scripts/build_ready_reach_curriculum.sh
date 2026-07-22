#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
base_build="${1:-$repo_root/training/builds/DG5F-VDI-builds-linux-x86_64-20260722/DG5FGraspReadyReach}"
output_build="${2:-$repo_root/training/builds/DG5FGraspReadyReachCurriculum}"
unity_root="${UNITY_ROOT:-/opt/unity/6000.4.0f1/Editor/Data}"
mono="$unity_root/MonoBleedingEdge/bin-linux64/mono"
csc="$unity_root/MonoBleedingEdge/lib/mono/4.5/csc.exe"
cecil="$unity_root/MonoBleedingEdge/lib/mono/gac/Mono.Cecil/0.11.1.0__0738eb9f132ed756/Mono.Cecil.dll"
policy_dir="$repo_root/training/policy/ready_reach"

base_managed="$base_build/DG5FGraspReadyReach_Data/Managed"
output_managed="$output_build/DG5FGraspReadyReach_Data/Managed"

for required in \
    "$base_build/DG5FGraspReadyReach.x86_64" \
    "$base_managed/KDT.ReachTraining.dll" \
    "$base_managed/Unity.ML-Agents.dll" \
    "$base_managed/UnityEngine.CoreModule.dll" \
    "$base_managed/netstandard.dll" \
    "$mono" "$csc" "$cecil"; do
    if [[ ! -e "$required" ]]; then
        echo "[ERROR] Required file is missing: $required" >&2
        exit 1
    fi
done

if [[ -e "$output_build" ]]; then
    echo "[ERROR] Output already exists; refusing to overwrite: $output_build" >&2
    exit 1
fi

work_dir="$(mktemp -d /tmp/dg5f-ready-reach-curriculum.XXXXXX)"
trap 'find "$work_dir" -type f -delete 2>/dev/null || true; find "$work_dir" -depth -type d -empty -delete 2>/dev/null || true' EXIT

"$mono" "$csc" -nologo -target:library \
    -out:"$work_dir/KDT.ReadyReachCurriculum.dll" \
    -reference:"$base_managed/netstandard.dll" \
    -reference:"$base_managed/KDT.ReachTraining.dll" \
    -reference:"$base_managed/Unity.ML-Agents.dll" \
    -reference:"$base_managed/UnityEngine.CoreModule.dll" \
    "$policy_dir/ReadyReachCurriculum.cs"

"$mono" "$csc" -nologo -target:exe \
    -out:"$work_dir/PatchReadyReach.exe" \
    -reference:"$cecil" \
    "$policy_dir/PatchReadyReach.cs"
cp "$cecil" "$work_dir/Mono.Cecil.dll"

mkdir -p "$output_build"
cp -r "$base_build"/. "$output_build"/
"$mono" "$work_dir/PatchReadyReach.exe" \
    "$base_managed/KDT.ReachTraining.dll" \
    "$work_dir/KDT.ReadyReachCurriculum.dll" \
    "$work_dir/KDT.ReachTraining.dll"
cp "$work_dir/KDT.ReachTraining.dll" \
    "$output_managed/KDT.ReachTraining.dll"
cp "$work_dir/KDT.ReadyReachCurriculum.dll" \
    "$output_managed/KDT.ReadyReachCurriculum.dll"
chmod +x "$output_build/DG5FGraspReadyReach.x86_64"

cat > "$output_build/curriculum-contract.txt" <<EOF
behavior=DG5FGraspReadyReach
observations=37
continuous_actions=6
parameter=reach_stage
stage_1=distance:0.05,speed:1000,hold:0.02
stage_2=distance:0.03,speed:0.15,hold:0.10
stage_3_optional=distance:0.01,speed:0.05,hold:0.25
base_reach_dll_sha256=$(sha256sum "$base_managed/KDT.ReachTraining.dll" | awk '{print $1}')
patched_reach_dll_sha256=$(sha256sum "$output_managed/KDT.ReachTraining.dll" | awk '{print $1}')
curriculum_dll_sha256=$(sha256sum "$output_managed/KDT.ReadyReachCurriculum.dll" | awk '{print $1}')
EOF

echo "[OK] ReadyReach curriculum build: $output_build"
cat "$output_build/curriculum-contract.txt"
