import math
from math import sin as sin
from math import cos as cos

from .bearing import Bearing

EARTH_RADIUS = 6371009  # in meters


class Point(object):
    '''A point on the face of the earth'''
    def __init__(self, latitude, longitude):
        self._lat = latitude
        self._long = longitude

    @classmethod
    def from_radians(cls, lat_radians, long_radians):
        '''
        Return a new instance of Point from a pair of coordinates in radians.
        '''
        return cls(math.degrees(lat_radians), math.degrees(long_radians))

    def __getitem__(self, key):
        if key == 0:
            return self._lat
        elif key == 1:
            return self._long
        else:
            raise IndexError('Point objects can only have two coordinates')

    def __iter__(self):
        '''Return an iterator containing the lat and long'''
        return iter([self.lat, self.long])

    def __str__(self):
        '''Return a string representation of the point'''
        return '{0:0.2f}N, {1:0.2f}W'.format(*list(self))

    def __repr__(self):
        return '<{0}.{1} ({2}) object at {3}>'.format(
            self.__module__, type(self).__name__, str(self), hex(id(self)))

    @property
    def lat(self):
        '''Return the latitude in degrees'''
        return self._lat

    @property
    def long(self):
        '''Return the longitude in degrees'''
        return self._long

    @property
    def lat_radians(self):
        '''Return the latitude in radians'''
        return math.radians(self.lat)

    @property
    def long_radians(self):
        '''Return the longitude in radians'''
        return math.radians(self.long)

    def distance_to(self, point):
        '''
        Return the distance between this point and another point in meters.

        :param point: Point to measure distance to
        :type point: Point

        :returns: The distance to the other point
        :rtype: float
        '''
        angle = math.acos(
            sin(self.lat_radians) * sin(point.lat_radians) +
            cos(self.lat_radians) * cos(point.lat_radians) *
            cos(self.long_radians - point.long_radians)
        )
        return angle * EARTH_RADIUS

    def bearing_to(self, point):
        '''
        Return the bearing to another point.

        :param point: Point to measure bearing to
        :type point: Point

        :returns: The bearing to the other point
        :rtype: Bearing
        '''
        delta_long = point.long_radians - self.long_radians
        y = sin(delta_long) * cos(point.lat_radians)
        x = (
            cos(self.lat_radians) * sin(point.lat_radians) -
            sin(self.lat_radians) * cos(point.lat_radians) * cos(delta_long)
        )
        radians = math.atan2(y, x)
        return Bearing.from_radians(radians)

    def cross_track_distance(self, start_point, end_point):
        '''
        Return the cross track distance from this point to the line between two
        points::

                        * end_point
                       /
                      /
                     /   * this point
                    /
                   /
                  *
             start_point


        :param start_point: First point on the line
        :type start_point: Point
        :param end_point: Second point on the line
        :type end_point: Point

        :returns: The perpendicular distance to the line between ``start_point`` 
                  and ``end_point``, where distance on the right of ``start_point``
                  is positive and distance on the left is negative 
        :rtype: float
        '''

        dist = start_point.distance_to(self)
        bearing_to_end = start_point.bearing_to(end_point).radians
        bearing_to_point = start_point.bearing_to(self).radians
        return math.asin(math.sin(dist / EARTH_RADIUS) * \
                         math.sin(bearing_to_point - bearing_to_end)) * \
                         EARTH_RADIUS

    def relative_point(self, bearing_to_point, distance):
        '''
        Return a waypoint at a location described relative to the current point

        :param bearing_to_point: Relative bearing from the current waypoint
        :param distance: Distance from the current waypoint
        :return: The point described by the parameters
        '''
        
        rad_distance = distance / EARTH_RADIUS
        target_long = math.asin((math.sin(self.lon_radians) * math.cos(rad_distance)) +
                                (math.cos(self.lon_radians) * math.sin(rad_distance) * math.cos(float(bearing_to_point))))

        a = math.sin(rad_distance) * math.sin(float(bearing_to_point))
        b = (math.cos(self.lon_radians) * math.cos(rad_distance)) - (math.sin(self.lon_radians) * math.sin(rad_distance) math.cos(float(bearing_to_point)))
        
        if b == 0:
            target_lat = self.lat_radians
        else:
            target_lat = self.lat_radians + math.atan2(a/b)

        target_lat = math.degrees(target_lat)
        target_long = math.degrees(target_long)

        return Point(target_lat, target_long)
    

    def __add__(self, other):
        return Point(self.lat + other.lat, self.long + other.long)

    def __sub__(self, other):
        return Point(self.lat - other.lat, self.long - other.long)

    def __div__(self, value):
        return Point(self.lat / value, self.long / value)



# do a couple of tests
if __name__ == '__main__':
    castle = Point(52.41389, -4.09098)  # aber castle
    print(castle)
    hill = Point(52.42459, -4.08339)  # Constitution hill
    print(hill)

    # distance should be ~1.29844 km
    print('regular:', castle.distance_to(hill))

    dismaland = Point(51.340911, -2.982787)
    print('regular:', castle.distance_to(dismaland))

    print('cross track:',
          Point(52.413990, -4.089979).cross_track_distance(castle, hill))

    print(castle.bearing_to(hill))

    # should be ~90 degrees
    print(Point(52.41398, -4.4627).bearing_to(Point(52.41398, -4.09122)))
