from functools import reduce


class ImmutableList(list):
    def __init__(self, d):
        list.__init__(self, d)
        if type(d) is dict: raise Exception
    def __item__(self, key):
        if key not in self:
            raise AttributeError(key)
        return self[key]
    def __setitem__(self, key, value):
        raise Exception


class ImmutableRecord(dict):
    def __init__(self, d):
        dict.__init__(self, d)
        if type(d) is list: raise Exception
    def __getattr__(self, key):
        if key not in self:
            raise AttributeError(key)
        return self[key]
    def __setattr__(self, key, value):
        raise Exception

#
# class MutableRecord(dict):
#     def __init__(self, d):
#         dict.__init__(self, d)
#         if type(d) is list: raise Exception
#     def __getattr__(self, key):
#         if key not in self:
#             raise AttributeError(key)
#         return self[key]
#     def __setattr__(self, key, value):
#         self[key] = value


def assoc(immutable_record, k, v):
    r = type(immutable_record).__bases__[0](immutable_record)
    r[k] = v
    return type(immutable_record)(r)


def assoc_in(immutable_struct, k_path, v):
    return _apply_in(immutable_struct, k_path, v, assoc)


def update_in(immutable_struct, k_path, f):
    return _apply_in(immutable_struct, k_path, f, update)


def _apply_in(immutable_struct, k_path, p, update_function):
    if len(k_path) == 0: return immutable_struct
    if len(k_path) == 1: return update_function(immutable_struct, k_path[0], p)

    def go(immutable_struct_, key_path):
        if len(key_path) == 2:
            tmp = update_function(immutable_struct_[key_path[0]], key_path[1], p)
            return assoc(immutable_struct_, key_path[0], tmp)

    return go(immutable_struct, k_path)


def update(immutableRecord, k, f):
    r = dict(immutableRecord)
    r[k] = f(r[k])
    return type(immutableRecord)(r)


def transact(record_or_record_and_f, *args):
    if type(record_or_record_and_f) is tuple:
        return reduce(
            lambda acc, val: record_or_record_and_f[1](acc, *val),
            args,
            record_or_record_and_f[0])
    else:
        return reduce(
            lambda acc, val: val[0](acc, *val[1:]),
            args,
            record_or_record_and_f)