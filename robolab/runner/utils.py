from immutable.immutable import ImmutableRecord


## general

find = lambda pred, default = 0: lambda l: next((i for i in l if pred(i)), default)


identity = lambda v: v


nop = lambda *_: None


lambdaize = lambda f: lambda *_: lambda: f(*_)


## more specific

minus = lambda l: lambda r: r - l


plus = lambda l: lambda r: r + l


sameIdentifier = lambda identifier: lambda item: 'identifier' in item and item['identifier'] == identifier


findByIdentifier = lambda identifier: find(sameIdentifier(identifier))


class XYZ(ImmutableRecord):
    def as_tuple(self):
        return (self.x, self.y, self.z)