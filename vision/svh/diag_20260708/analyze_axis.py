import csv, math
from collections import defaultdict

path = r"C:\Users\dltmd\AppData\Local\Temp\claude\C--Users-dltmd-Desktop-KDT\cdbb52d1-44f1-4244-a998-555745a3a6f7\scratchpad\axis_samples.csv"
rows = defaultdict(list)
with open(path) as f:
    for r in csv.DictReader(f):
        rows[r["link"]].append((float(r["t"]), float(r["wmag"]), float(r["angleFolded"]), float(r["jointPosDeg"]), float(r["targetDeg"])))

def wq(pairs, q):  # weighted quantile of angle by wmag
    s = sorted(pairs); tot = sum(w for _, w in s); acc = 0
    for a, w in s:
        acc += w
        if acc >= q * tot: return a
    return s[-1][0] if s else float("nan")

print(f"{'link':22s} {'n_mov':>5s} {'wmax':>7s} {'w_mean':>7s} | {'angW':>6s} {'ang50':>6s} {'ang95':>6s} | {'jp_min':>8s} {'jp_max':>8s} {'jp_p2p':>7s} | {'tgt_max':>7s}")
for link in ["right_hand_z","right_hand_virtual_i","right_hand_l","right_hand_p","right_hand_k","right_hand_o"]:
    d = rows[link]
    mov = [(a, w) for _, w, a, _, _ in d if w > 0.05 and a >= 0]
    jps = [jp for _, _, _, jp, _ in d]
    tgts = [tg for *_, tg in d]
    wmax = max(w for _, w, *_ in d)
    if mov:
        wsum = sum(w for _, w in mov)
        angw = sum(a * w for a, w in mov) / wsum
        wmean = wsum / len(mov)
        a50, a95 = wq(mov, 0.5), wq(mov, 0.95)
    else:
        angw = wmean = a50 = a95 = float("nan")
    print(f"{link:22s} {len(mov):5d} {wmax:7.2f} {wmean:7.2f} | {angw:6.1f} {a50:6.1f} {a95:6.1f} | {min(jps):8.2f} {max(jps):8.2f} {max(jps)-min(jps):7.2f} | {max(tgts):7.2f}")

# steady-state (last 3 s, target should be 0/held): oscillation check
print("\n-- last 3s (after probe ended, command held) --")
tmax = max(t for d in rows.values() for t, *_ in d)
for link in ["right_hand_z","right_hand_virtual_i","right_hand_l","right_hand_p","right_hand_k","right_hand_o"]:
    d = [x for x in rows[link] if x[0] > tmax - 3.0]
    jps = [jp for _, _, _, jp, _ in d]
    mov = [(a, w) for _, w, a, _, _ in d if w > 0.05 and a >= 0]
    angw = (sum(a*w for a,w in mov)/sum(w for _,w in mov)) if mov else float("nan")
    wmax = max(w for _, w, *_ in d) if d else 0
    print(f"{link:22s} jp_p2p={max(jps)-min(jps):8.3f}  wmax={wmax:7.3f}  angW={angw:6.1f}  n_mov={len(mov)}")
