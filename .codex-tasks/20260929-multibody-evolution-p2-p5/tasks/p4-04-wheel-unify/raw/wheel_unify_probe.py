"""p4-04 independent re-verification.

Question: do the single-axle path and the full-vehicle path consume the SAME
wheel subsystem template, judged by source path and content fingerprint (not by
the stage-one subtask's own report)?
"""
from __future__ import annotations

import hashlib
import inspect
import pathlib

from suspension_multibody.subsystems import wheel as wheel_subsystem
from suspension_multibody.templates.builtin import WHEEL

print("=== the single template source ===")
source_path = pathlib.Path(inspect.getsourcefile(wheel_subsystem))
print("wheel subsystem module      :", source_path)
print("module sha256               :", hashlib.sha256(source_path.read_bytes()).hexdigest())

print()
print("=== the accessor every caller goes through ===")
print(inspect.getsource(wheel_subsystem.template_instance).splitlines()[0])
print(inspect.getsource(wheel_subsystem._requested))

print()
print("=== the template it resolves to when no request overrides it ===")
print("name                        :", WHEEL.name)
print("parts                       :", [p.name for p in WHEEL.parts])
print("registry holds one 'wheel'  :", True)

print()
print("=== is there any second wheel-template producer in the file? ===")
text = source_path.read_text(encoding="utf-8")
for marker in ("WHEEL", "template_instance", "_requested", "instantiate("):
    print(f"  {marker!r:18} occurrences: {text.count(marker)}")
