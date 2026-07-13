"""UR5e(flat) + SVH right hand -> 단일 ur5e_svh_right.urdf 결합.

규칙:
- SVH의 base_link -> svh_base_link 로 rename (링크 정의 + 이를 참조하는 joint parent/child)
- UR tool0 --(fixed)--> svh_base_link 연결 (origin 0 0 0)
- mesh 경로 통일:
    UR : package://ur_description/meshes/...  -> meshes/ur/...
    SVH: meshes/...                            -> meshes/svh/...
- mimic joint 보존
- robot name = ur5e_svh
"""
import lxml.etree as ET

BUILD = r"C:/Users/dltmd/Desktop/KDT/ur5e_svh_build"
UR = BUILD + "/ur5e_raw.urdf"
SVH = r"C:/Users/dltmd/Desktop/KDT/dex-urdf-main/robots/hands/schunk_hand/schunk_svh_hand_right.urdf"
OUT = BUILD + "/ur5e_svh_right.urdf"

parser = ET.XMLParser(remove_blank_text=True)
ur_root = ET.parse(UR, parser).getroot()
svh_root = ET.parse(SVH, parser).getroot()


def rewrite_mesh(root, prefix_from, prefix_to):
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn and fn.startswith(prefix_from):
            mesh.set("filename", prefix_to + fn[len(prefix_from):])


# 1) mesh 경로 통일
rewrite_mesh(ur_root, "package://ur_description/meshes/", "meshes/ur/")
rewrite_mesh(svh_root, "meshes/", "meshes/svh/")

# 2) SVH base_link -> svh_base_link rename
OLD, NEW = "base_link", "svh_base_link"
for link in svh_root.findall("link"):
    if link.get("name") == OLD:
        link.set("name", NEW)
for joint in svh_root.findall("joint"):
    for tag in ("parent", "child"):
        e = joint.find(tag)
        if e is not None and e.get("link") == OLD:
            e.set("link", NEW)

# 3) 새 robot 루트 구성
new_root = ET.Element("robot", name="ur5e_svh")

seen_materials = set()


def append_children(src):
    for child in src:
        if child.tag == "material":
            nm = child.get("name")
            if nm in seen_materials:
                continue
            seen_materials.add(nm)
        new_root.append(child)


append_children(ur_root)   # world ~ tool0
append_children(svh_root)  # svh_base_link ~ fingertips

# 4) tool0 -> svh_base_link 연결 (fixed)
conn = ET.SubElement(new_root, "joint", name="tool0_to_svh", type="fixed")
ET.SubElement(conn, "parent", link="tool0")
ET.SubElement(conn, "child", link="svh_base_link")
ET.SubElement(conn, "origin", xyz="0 0 0", rpy="0 0 0")

# 5) 저장 (UTF-8)
tree = ET.ElementTree(new_root)
tree.write(OUT, pretty_print=True, xml_declaration=True, encoding="utf-8")

# ---------------- 검증 ----------------
links = [l.get("name") for l in new_root.findall("link")]
joints = new_root.findall("joint")
print("robot name:", new_root.get("name"))
print("links:", len(links), "| joints:", len(joints))

# 트리 연결성: world -> ... -> fingertips
edges = {}
for j in joints:
    p = j.find("parent").get("link")
    c = j.find("child").get("link")
    edges.setdefault(p, []).append(c)

reached = set()
stack = ["world"]
while stack:
    n = stack.pop()
    if n in reached:
        continue
    reached.add(n)
    stack.extend(edges.get(n, []))

linkset = set(links)
unreached = linkset - reached
print("reachable from 'world':", len(reached & linkset), "/", len(linkset))
print("unreachable links:", sorted(unreached) if unreached else "NONE")
for tip in ["fftip", "thtip", "mftip", "rftip", "lftip"]:
    print(f"  tip {tip}: {'reached' if tip in reached else 'NOT reached'}")

# mimic 보존 확인
mimics = new_root.findall(".//joint/mimic")
print("mimic joints preserved:", len(mimics))

# mesh refs
meshes = [m.get("filename") for m in new_root.iter("mesh")]
print("total mesh refs:", len(meshes), "| unique:", len(set(meshes)))
print("OUTPUT:", OUT)
