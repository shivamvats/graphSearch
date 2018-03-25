from heuristicSearch.utils.utils import euclideanDistance

class CircularRegion():
    """Models an arbitrary continuous shape/region. Center is a 2D coordinate
    and radius is a scalar.
    * Implements a `contains` method."""

    def __init__(self, center=None, radius=None):
        if center is None or radius is None:
            raise ValueError
        self._center = center
        self._radius = radius

    @property
    def center(self):
        return self._center

    @property
    def radius(self):
        return self._radius

    @center.setter
    def center(self, center):
        self._center = center

    @radius.setter
    def radius(self, radius):
        self._radius = radius

    def contains(self, point):
        if euclideanDistance(self._center, point) < self._radius:
            return True
        else:
            return False

    def boundaryContains(self, point, neighbours):
        if self.contains(point) and any([not self.contains(neighbour) for
                neighbour in neighbours]):
            return True
        else:
            return False

    def interiorContains(self, point, neighbours):
        if self.contains(point) and not self.boundaryContains(point,
                neighbours):
            return True
        else:
            return False



