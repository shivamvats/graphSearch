from circular_region import CircularRegion

class IslandRegion():
    """Models an island with an association influence region.
    An island is stored as a Node and its corresponding influence region is
    stored by a region class.
    * Implements `contains` method that checks if a given point is
    inside the region or not.
    * An island is a Node.
    * The region used coordinates."""

    def __init__(self, island=None, region=None):
        if island is None or region is None:
            raise ValueError
        self._island = island
        self._region = region

    @property
    def island(self):
        return self._island

    @island.setter
    def island(self, island):
        self._island = island

    @property
    def region(self):
        return self._region

    @property
    def inflation(self):
        return self._inflation

    @inflation.setter
    def inflation(self, inflation):
        self._inflation = inflation

    def contains(self, point):
        """Assumes `point` is a 2D coordinate."""
        return self._region.contains(point)
