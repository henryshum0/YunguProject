import math
import random
import unittest

from benchmark.generate_map import build_sdf, make_obstacle


class SuspendedSlabTest(unittest.TestCase):
    def setUp(self):
        self.gate_cfg = {
            "gates": {
                "enabled": True,
                "width_min": 4.0,
                "width_max": 4.0,
                "center_height_min": 5.0,
                "center_height_max": 5.0,
                "slab_thickness": 0.4,
                "slab_depth": 0.6,
            },
            "pillars": {"enabled": False},
        }

    def test_gate_uses_slab_bounding_radius(self):
        gate = make_obstacle(random.Random(0), self.gate_cfg)

        self.assertEqual(gate["kind"], "gate")
        self.assertEqual(gate["center_height"], 5.0)
        self.assertAlmostEqual(gate["half"], math.hypot(4.0, 0.6) / 2.0)

    def test_sdf_contains_one_centered_slab(self):
        gate = make_obstacle(random.Random(0), self.gate_cfg)
        gate.update({"x": 2.0, "y": 3.0, "yaw": 0.0})
        sdf = build_sdf({"world_name": "test"}, [gate], 10.0, 10.0)

        self.assertIn('<collision name="slab">', sdf)
        self.assertIn('<visual name="slab_vis">', sdf)
        self.assertIn('<pose>0 0 5.000 0 0 0</pose>', sdf)
        self.assertIn('<box><size>4.000 0.600 0.400</size></box>', sdf)
        self.assertNotIn("post_l", sdf)
        self.assertNotIn("post_r", sdf)


if __name__ == "__main__":
    unittest.main()
