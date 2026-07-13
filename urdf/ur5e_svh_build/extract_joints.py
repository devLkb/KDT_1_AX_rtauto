"""결합 URDF에서 mimic/구동/소스 관절의 child link 이름을 추출해 JSON으로."""
import lxml.etree as ET
import json

URDF = r"C:/Users/dltmd/Desktop/KDT/ur5e_svh_build/unity_pkg/ur5e_svh_right.urdf"
root = ET.parse(URDF).getroot()

joints = {}
for j in root.findall("joint"):
    name = j.get("name")
    child = j.find("child").get("link")
    jtype = j.get("type")
    mimic = j.find("mimic")
    info = {"child": child, "type": jtype}
    if mimic is not None:
        info["mimic_source"] = mimic.get("joint")
        info["multiplier"] = float(mimic.get("multiplier", "1"))
        info["offset"] = float(mimic.get("offset", "0"))
    lim = j.find("limit")
    if lim is not None:
        info["lower"] = float(lim.get("lower", "0"))
        info["upper"] = float(lim.get("upper", "0"))
    joints[name] = info

# 구동 관절 9개
driven = [
    "right_hand_Thumb_Flexion", "right_hand_Thumb_Opposition",
    "right_hand_Index_Finger_Proximal", "right_hand_Index_Finger_Distal",
    "right_hand_Middle_Finger_Proximal", "right_hand_Middle_Finger_Distal",
    "right_hand_Ring_Finger", "right_hand_Pinky", "right_hand_Finger_Spread",
]

mimics = {n: i for n, i in joints.items() if "mimic_source" in i}

out = {
    "driven": {n: joints[n] for n in driven},
    "mimics": mimics,
}
print(json.dumps(out, indent=2))

# C# 친화적 요약: mimic = (mimicChildLink, sourceChildLink, mult, offset)
print("\n=== MIMIC ROWS (mimicChild | sourceChild | mult | offset) ===")
for n, i in mimics.items():
    src = i["mimic_source"]
    src_child = joints[src]["child"]
    print(f'{i["child"]} | {src_child} | {i["multiplier"]} | {i["offset"]}')

print("\n=== DRIVEN ROWS (joint | child | lower | upper) ===")
for n in driven:
    i = joints[n]
    print(f'{n} | {i["child"]} | {i.get("lower")} | {i.get("upper")}')
