#!/usr/bin/env python3
"""check_gunon_calls.py - does every spray stroke of every program really switch the gun ON and OFF?

  python3 check_gunon_calls.py                 all programs named Auto_*  (except Auto_Main)
  python3 check_gunon_calls.py Auto_00_hood    only the named programs

Prints, per program: the number of moves and of GunOn ON / OFF calls, and the first part of the sequence, e.g.
   ON | 84 moves | OFF | 12 moves | ON | 90 moves | OFF ...
Every stroke should look like  ON | many moves | OFF.  Anything else (an ON without an OFF, strokes without ON) shows why paint is missing.
"""
import re
import sys
from robodk import robolink

MOVE = getattr(robolink, "INS_TYPE_MOVE", 0)


def classify(name, instype):
    n = (name or "").lower().replace(" ", "")
    m = re.search(r"gunon\(([-+]?\d+)", n)
    if m:
        v = int(m.group(1))
        return "CLEAR" if v < 0 else ("OFF" if v == 0 else ("STATS" if v == 2 else "ON"))
    return "move" if instype == MOVE else "other"


def summarise(prog):
    seq = []
    for i in range(prog.InstructionCount()):
        try:
            ins = prog.Instruction(i)
            kind = classify(ins[0], ins[1])
        except Exception:
            continue
        if kind == "other":
            continue
        if kind == "move" and seq and isinstance(seq[-1], int):
            seq[-1] += 1
        else:
            seq.append(1 if kind == "move" else kind)
    return seq


def main():
    RDK = robolink.Robolink()
    names = sys.argv[1:] or sorted(n for n in RDK.ItemList(robolink.ITEM_TYPE_PROGRAM, True)
                                    if n.startswith("Auto_") and n != "Auto_Main" and not n.startswith("Auto_Main"))
    for nm in names:
        prog = RDK.Item(nm, robolink.ITEM_TYPE_PROGRAM)
        if not prog.Valid():
            print(f"{nm}: not found")
            continue
        seq = summarise(prog)
        n_on, n_off = seq.count("ON"), seq.count("OFF")
        moves = sum(x for x in seq if isinstance(x, int))
        text = " | ".join((f"{x} moves" if isinstance(x, int) else x) for x in seq[:14]) + (" ..." if len(seq) > 14 else "")
        flag = "" if (n_on == n_off and n_on > 0) else "   <-- CHECK: ON and OFF counts differ or there is no spray"
        print(f"{nm}: {moves} moves, ON x{n_on}, OFF x{n_off}{flag}\n    {text}")


if __name__ == "__main__":
    main()
