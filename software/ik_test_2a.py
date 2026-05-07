from ik_solver_2 import IKController
from unittest.mock import MagicMock

# Mock the arm so no hardware is needed
arm = MagicMock()
arm.BASE     = 0
arm.SHOULDER = 1
arm.ELBOW    = 2
arm.CLAW     = 4

controller = IKController(arm)

passed      = 0
failed      = 0
unreachable = 0

for x in range(130, 370, 10):       # horizontal range in mm
    for y in range(-200, 200, 10):  # lateral range in mm
        try:
            angles = controller._ik(x, y, controller.Z, elbow_up=True)
            controller._verify(angles, x, y, controller.Z)
            passed += 1
        except AssertionError as e:
            print(f"FK MISMATCH at ({x}, {y}): {e}")
            failed += 1
        except ValueError:
            unreachable += 1

print(f"\nPassed:      {passed}")
print(f"FK mismatches: {failed}")
print(f"Unreachable: {unreachable}")

import ik_solver
print(ik_solver.__file__)