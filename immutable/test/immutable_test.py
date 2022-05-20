import unittest

from immutable.immutable import *
from runner.utils import XYZ # todo get rid of dependency


class TestImmutable(unittest.TestCase):

    def test_transact(self):

        r1 = ImmutableRecord({'x': 5, 'y': 7, 'z': 9})
        r2 = transact(r1,
                      (assoc, 'x', 6),
                      (assoc, 'y', 8))

        self.assertEqual(r1.x, 5)
        self.assertEqual(r1.y, 7)
        self.assertEqual(r2.x, 6)
        self.assertEqual(r2.y, 8)


    def test_transact_with_short_form(self):

        r1 = ImmutableRecord({'x': 5, 'y': 7, 'z': 9})
        r2 = transact((r1, assoc), ('x', 6), ('y', 8))

        self.assertEqual(r1.x, 5)
        self.assertEqual(r1.y, 7)
        self.assertEqual(r2.x, 6)
        self.assertEqual(r2.y, 8)



    def test_create_immutable(self):
        r = ImmutableRecord({'x': 6, 'y': 7})
        self.assertEqual(r.x, 6)
        self.assertEqual(r.y, 7)

        r = ImmutableList([8, 9])
        self.assertEqual(r[0], 8)
        self.assertEqual(r[1], 9)


    def test_correct_types(self):
        with self.assertRaises(Exception):
            ImmutableRecord([])
        with self.assertRaises(Exception):
            ImmutableList({})


    def test_is_immutable(self):
        r = ImmutableRecord({'x': 7})
        with self.assertRaises(Exception):
            r.x = 8
        l = ImmutableList([5, 6])
        with self.assertRaises(Exception):
            l[0] = 9


    def test_raise_if_item_not_exist(self):
        r = ImmutableRecord({'a': 3})
        existing = r.a
        with self.assertRaises(Exception):
            print(existing + r.b)

        l = ImmutableList([5, 6])
        existing = l[0]
        with self.assertRaises(Exception):
            print(existing + l[3])


    def test_make_mutable(self):
        r = dict(ImmutableRecord({'x': 7}))
        r['x'] = 8
        self.assertEqual(r['x'], 8)

        l = list(ImmutableList([4, 5]))
        l[0] = 6
        self.assertEqual(l[0], 6)


    def test_assoc(self):
        r1 = ImmutableRecord({'x': 7, 'y': 9})
        r2 = assoc(r1, 'x', 8)
        self.assertEqual(r1.x, 7) # r1 still unchanged
        self.assertEqual(r2.x, 8) # r2 with changed element
        self.assertEqual(r2.y, 9) # r2 with other elements from r1
        with self.assertRaises(Exception):
            r2.x = 9 # r2 is ImmutableRecord

        l1 = ImmutableList([3, 4])
        l2 = assoc(l1, 0, 8)
        self.assertEqual(l1[0], 3) # l1 still unchanged
        self.assertEqual(l2[0], 8) # l2 with changed element
        self.assertEqual(l2[1], 4) # l2 with other elements from l1
        with self.assertRaises(Exception):
            l2[0] = 9 # l2 is ImmutableList


    def test_can_be_subclassed_and_assoc_retains_type(self):
        class SubclassedImmutableRecord(ImmutableRecord):
            def existing_method(self):
                return 7
        s1 = SubclassedImmutableRecord({'a': 9})
        s2 = assoc(s1, 'a', 5)
        result = s2.existing_method()
        self.assertEqual(result, 7) # can call method which only exists in subclass

        self.assertEqual(result, 7)
        with self.assertRaises(Exception):
            dict(s1).existing_method() # which is not present anymore if converted back to dict


    def test_update(self):
        r1 = ImmutableRecord({'x': 7})
        r2 = update(r1, 'x', lambda v: v + 1)
        self.assertEqual(r2.x, 8)
        with self.assertRaises(Exception):
            r2.x = 9


    def test_has_tuple_method_after_calling_update(self):
        r1 = XYZ({'x': 7, 'y': 8, 'z': 9})
        self.assertEqual(r1.as_tuple()[0], 7)
        r2 = update(r1, 'x', lambda v: v + 1)
        self.assertEqual(r2.as_tuple()[0], 8)


    def test_assoc_in(self):
        t1 = ImmutableRecord({'a': ImmutableRecord({'b': 3})})
        t2 = assoc_in(t1, ['a', 'b'], 7)
        self.assertEqual(t1.a.b, 3)
        self.assertEqual(t2.a.b, 7)

        t1 = ImmutableRecord({'a': 3})
        t2 = assoc_in(t1, ['a'], 7)
        self.assertEqual(t1.a, 3)
        self.assertEqual(t2.a, 7)

        t1 = ImmutableRecord({'a': 3})
        t2 = assoc_in(t1, [], 7)
        self.assertEqual(t1.a, 3)
        self.assertEqual(t2.a, 3)


    def test_update_in(self):
        t1 = ImmutableRecord({'a': ImmutableRecord({'b': 3})})
        t2 = update_in(t1, ['a', 'b'], lambda v: v + 1)
        self.assertEqual(t1.a.b, 3)
        self.assertEqual(t2.a.b, 4)

        t1 = ImmutableRecord({'a': 3})
        t2 = update_in(t1, ['a'], lambda v: v + 1)
        self.assertEqual(t1.a, 3)
        self.assertEqual(t2.a, 4)

        t1 = ImmutableRecord({'a': 3})
        t2 = assoc_in(t1, [], lambda v: v + 1)
        self.assertEqual(t1.a, 3)
        self.assertEqual(t2.a, 3)


if __name__ == '__main__':
    unittest.main()