import math
import sys
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

rospy_stub = types.ModuleType("rospy")
rospy_stub.get_param = lambda _name, default=None: default
rospy_stub.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0))
sys.modules.setdefault("rospy", rospy_stub)

std_msgs_stub = types.ModuleType("std_msgs")
std_msgs_msg_stub = types.ModuleType("std_msgs.msg")
std_msgs_msg_stub.Bool = type("Bool", (), {})
std_msgs_msg_stub.String = type("String", (), {})
sys.modules.setdefault("std_msgs", std_msgs_stub)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg_stub)

from stage1_vision_node import apply_ring_camera_mount_offsets


class RingGateOffsetTests(unittest.TestCase):
    def test_mount_offsets_convert_camera_measurement_to_body_center_offset(self):
        forward, left, up = apply_ring_camera_mount_offsets(
            forward_m=1.20,
            offset_y_m=-0.04,
            offset_z_m=0.03,
            camera_forward_offset_m=0.10,
            camera_left_offset_m=0.06,
            camera_up_offset_m=-0.02,
        )

        self.assertTrue(math.isclose(forward, 1.30, abs_tol=1e-9))
        self.assertTrue(math.isclose(left, 0.02, abs_tol=1e-9))
        self.assertTrue(math.isclose(up, 0.01, abs_tol=1e-9))


if __name__ == "__main__":
    unittest.main()
