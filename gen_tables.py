#!/usr/bin/env python3
"""dvtables.h: the V3 air format's tables for the firmware, from the reference (dvframe.py)."""
import numpy as np, dvframe as d
F = d.V3
with open("dvtables.h", "w") as f:
    f.write(f"/* {F.name} framing tables (gen_tables.py from dvframe.py) */\n")
    f.write(f"#define SYNC_LEN {d.SYNC_LEN}\n#define FRAME_SYMS {F.FRAME_SYMS}\n#define DATA_SYMS {F.DATA_SYMS}\n")
    f.write(f"#define SF_PER_FRAME {F.SF_PER_FRAME}\n#define INFO {F.INFO}\n#define CODED {F.CODED}\n#define KEPT {2 * F.DATA_SYMS}\n")
    f.write("static const uint8_t sync_bits[SYNC_LEN] = {" + ",".join(str(int(b)) for b in d.SYNC) + "};\n")
    f.write("static const uint8_t fmt_order[INFO] = {" + ",".join(str(int(v)) for v in F.ORDER) + "};   /* info bit i = payload bit */\n")
    f.write("static const uint8_t fmt_kept[KEPT] = {" + ",".join(str(int(v)) for v in np.flatnonzero(F.KEEP)) + "};   /* kept coded bits */\n")
    f.write("static const uint8_t fmt_ilv[KEPT] = {" + ",".join(str(int(v)) for v in F.ILV) + "};   /* kept bit k -> position */\n")
print("wrote dvtables.h")
