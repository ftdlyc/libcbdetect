from cbdetect import add
from unittest import TestCase

class TestAdd(TestCase):
    def test_add(self):
        assert add(1, 2) == 3
