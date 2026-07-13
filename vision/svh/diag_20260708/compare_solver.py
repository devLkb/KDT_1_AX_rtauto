import csv
from collections import defaultdict

SP = r"C:\Users\dltmd\AppData\Local\Temp\claude\C--Users-dltmd-Desktop-KDT\cdbb52d1-44f1-4244-a998-555745a3a6f7\scratchpad"

def load(name):
    rows = defaultdict(list)
    with open(SP + "\\" + name) as f:
        for r in csv.DictReader(f):
            rows[r["link"]].append((float(r["t"]), float(r["jointPosDeg"]), float(r["targetDeg"]), float(r["lowerDeg"])))
    return rows

def metrics(d):
    tmax = max(t for t, *_ in d)
    jps = [jp for _, jp, _, _ in d]
    last3 = [x for x in d if x[0] > tmax - 3.0]
    jps3 = [jp for _, jp, _, _ in last3]
    neg = [jp for _, jp, _, lo in d if jp < lo - 1.0]
    errs = [abs(jp - tg) for _, jp, tg, _ in d]
    errs3 = [abs(jp - tg) for _, jp, tg, _ in last3]
    return dict(
        p2p_all=max(jps) - min(jps),
        p2p_ss=max(jps3) - min(jps3),
        neg_pct=100.0 * len(neg) / len(d),
        neg_min=min(neg) if neg else 0.0,
        err_all=sum(errs) / len(errs),
        err_ss=sum(errs3) / len(errs3),
    )

before, after = load("before_pgs.csv"), load("after_tgs.csv")
links = ["right_hand_z", "right_hand_virtual_i", "right_hand_l", "right_hand_p", "right_hand_k", "right_hand_o"]
hdr = ("link", "p2p_all", "p2p_ss", "neg%", "neg_min", "|err|all", "|err|ss")
print(f"{'':22s} {'── PGS(before) ──':^52s} | {'── TGS(after) ──':^52s}")
print(f"{'link':22s} " + " ".join(f"{h:>8s}" for h in hdr[1:]) + " | " + " ".join(f"{h:>8s}" for h in hdr[1:]))
for l in links:
    b, a = metrics(before[l]), metrics(after[l])
    fmt = lambda m: f"{m['p2p_all']:8.2f} {m['p2p_ss']:8.2f} {m['neg_pct']:8.1f} {m['neg_min']:8.2f} {m['err_all']:8.2f} {m['err_ss']:8.2f}"
    print(f"{l:22s} {fmt(b)} | {fmt(a)}")

# fist-phase tracking: mean |err| where target > 5 deg
print("\nfist-phase |act-tgt| (target>5deg):")
for l in links:
    for tag, rows in (("PGS", before[l]), ("TGS", after[l])):
        e = [abs(jp - tg) for _, jp, tg, _ in rows if tg > 5]
        print(f"  {l:22s} {tag}: {sum(e)/len(e):7.2f}" if e else f"  {l:22s} {tag}: n/a", end="")
    print()
