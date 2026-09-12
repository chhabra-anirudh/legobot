import unittest
from cube_colors import rgb
from simulate_assembly import HERE, load_config


class ColorTests(unittest.TestCase):
    def test_known_palette(self):
        self.assertEqual(rgb('red'), (225, 65, 65))
        self.assertNotEqual(rgb('red'), rgb('blue'))

    def test_invalid_color_is_rejected(self):
        for name in ['invisible', None, []]:
            with self.assertRaises(ValueError):
                rgb(name)

    def test_assembly_has_a_color_per_cube(self):
        c = load_config(HERE/'assembly_config.json')
        self.assertEqual(c['block_colors'], ['red', 'green', 'blue'])
        self.assertEqual(len(c['block_colors']), len(c['supply_xy_m']))


if __name__ == '__main__':
    unittest.main()
