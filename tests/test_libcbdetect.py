from cbdetect_py import add
from unittest import TestCase
import numpy as np

class TestAdd(TestCase):
    def test_add(self):
        a = np.array([[1, 2]], np.int32)
        b = np.array([[2, 3]], np.int32)
        np.testing.assert_array_equal( add(a, b) , a + b)
