# Run the OndselSolver driver on every case and store the placements (see run.sh).
import json
import os
import subprocess
import sys

drv, here = sys.argv[1], sys.argv[2]
for out, ratio in (("ondsel_master.json", None), ("ondsel_r3.json", str(1.0 / 3.0))):
    env = dict(os.environ)
    if ratio:
        env["AJ_RATIO"] = ratio
    res = {}
    for f in sorted(os.listdir(os.path.join(here, "asmt"))):
        case = f.replace("_start.asmt", "")
        txt = subprocess.run([drv, os.path.join(here, "asmt", f)], capture_output=True, text=True, env=env, check=True).stdout
        parts = {}
        for line in txt.splitlines():
            if line.startswith("PART "):
                t = line.split()
                name = t[1].split("#", 1)[1]
                parts[name] = {"pos": [float(x) for x in t[3:6]], "matrix": [float(x) for x in t[7:16]]}
        res[case] = parts
    json.dump(res, open(os.path.join(here, out), "w"), indent=1)
    print("wrote", out)
