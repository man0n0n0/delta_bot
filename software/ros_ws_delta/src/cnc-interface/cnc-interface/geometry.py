class Point:
    def __init__(self, x=0, y=0, z=0, coordinate="3dPrinterHome"):
        self.x = x 
        self.y = y
        self.z = z
        self.coordinate = "3dPrinterHome"

    def __eq__(self, other) -> bool:
        if not isinstance(other, Point):
            return False
        pairs = ((self.x, other.x), (self.y, other.y), (self.z, other.z))
        return all(isclose(s, o,
                           rel_tol=1e-05, abs_tol=1e-08) for s, o in pairs)


    def __add__(self, other) -> 'Point':
        if not isinstance(other, Point):
            return NotImplemented
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other) -> 'Point':
        if not isinstance(other, Point):
            return NotImplemented
        return Point(self.x - other.x, self.y - other.y, self.z - other.z)

    def __abs__(self) -> 'Point':
        return Point(abs(self.x), abs(self.y), abs(self.z))

    def __str__(self):
        return '({}, {}, {})'.format(self.x, self.y, self.z)
