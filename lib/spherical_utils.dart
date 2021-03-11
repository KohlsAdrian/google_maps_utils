///  Copyright 2013 Google Inc.
///
///  Licensed under the Apache License, Version 2.0 (the 'License');
///  you may not use this file except in compliance with the License.
///  You may obtain a copy of the License at
///
///       http://www.apache.org/licenses/LICENSE-2.0
///
///  Unless required by applicable law or agreed to in writing, software
///  distributed under the License is distributed on an 'AS IS' BASIS,
///  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
///  See the License for the specific language governing permissions and
///  limitations under the License.
import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';

class GMULatLngBounds {
  final Point northEast;
  final Point southWest;

  GMULatLngBounds(this.northEast, this.southWest);
}

class SphericalUtils {
  SphericalUtils._();

  ///  Missing simple conversions from Math class
  ///  Code from: https://github.com/dart-lang/sdk/issues/4211#issue-84512743
  static double toRadians(num deg) => deg * (pi / 180.0);
  static double toDegrees(num rad) => rad * (180.0 / pi);

  /// see: https://stackoverflow.com/a/25867068/3182210
  static String getCardinal(double angle) {
    var val = ((angle / 22.5) + 0.5).floor();
    var arr = [
      'N',
      'NNE',
      'NE',
      'ENE',
      'E',
      'ESE',
      'SE',
      'SSE',
      'S',
      'SSW',
      'SW',
      'WSW',
      'W',
      'WNW',
      'NW',
      'NNW',
    ];
    return arr[(val % 16)];
  }

  /// see: https://stackoverflow.com/a/31029389/3182210
  static GMULatLngBounds toBounds(double x, double y, double radiusInMeters) {
    Point center = Point(x, y);
    double distanceFromCenterToCorner = radiusInMeters * sqrt(2.0);
    Point southwestCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 225.0);
    Point northeastCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 45.0);
    return GMULatLngBounds(northeastCorner, southwestCorner);
  }

  /// Convert a list of latitudes and longitudes to an object of boundaries
  ///
  /// see: https://github.com/flutter/flutter/issues/36653#issuecomment-525288053
  ///
  /// [return] The boundaries from the list of points
  static GMULatLngBounds toBoundsFromPoints(List<Point> points) {
    if (points.isEmpty) throw Exception("Points cannot be empty");

    double x0, x1, y0, y1;
    x0 = x1 = points.first.x.toDouble();
    y0 = y1 = points.first.y.toDouble();
    points.forEach((point) {
      if (point.x > x1) x1 = point.x.toDouble();
      if (point.x < x0) x0 = point.x.toDouble();
      if (point.y > y1) y1 = point.y.toDouble();
      if (point.y < y0) y0 = point.y.toDouble();
    });

    Point northEast = Point(x1, y1);
    Point southWest = Point(x0, y0);

    GMULatLngBounds gmuLatLngBounds = GMULatLngBounds(northEast, southWest);

    return gmuLatLngBounds;
  }

  /// Calculate the center of the boundaries to a Point in latitude and longitude
  ///
  /// see: https://stackoverflow.com/a/30859321/3182210
  ///
  /// [return] the Point representing the center of the [gmuLatLngBounds]
  static Point centerFromLatLngBounds(GMULatLngBounds gmuLatLngBounds) {
    Point northEast = gmuLatLngBounds.northEast;
    Point southWest = gmuLatLngBounds.southWest;

    double centerLatitudeBound = (northEast.x + southWest.x) / 2;
    double centerLongitudeBound = (northEast.y + southWest.y) / 2;

    Point center = Point(centerLatitudeBound, centerLongitudeBound);

    return center;
  }

  /// Returns the heading from one Point to another Point. Headings are
  /// expressed in degrees clockwise from North within the range [-180,180).
  ///
  /// [return] The heading in degrees clockwise from north.
  static double computeHeading(Point from, Point to) {
    // http://williams.best.vwh.net/avform.htm#Crs

    double fromLat = toRadians(from.x);
    double fromLng = toRadians(from.y);
    double toLat = toRadians(to.x);
    double toLng = toRadians(to.y);
    double dLng = toLng - fromLng;
    double heading = atan2(sin(dLng) * cos(toLat),
        cos(fromLat) * sin(toLat) - sin(fromLat) * cos(toLat) * cos(dLng));
    return MathUtils.wrap(toDegrees(heading), -180, 180);
  }

  /// Returns the Point resulting from moving a distance from an origin
  /// in the specified heading (expressed in degrees clockwise from north).
  ///
  /// [from]     The Point from which to start.
  ///
  /// [distance] The distance to travel.
  ///
  /// [heading]  The heading in degrees clockwise from north.
  static Point computeOffset(Point from, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://williams.best.vwh.net/avform.htm#LL
    double fromLat = toRadians(from.x);
    double fromLng = toRadians(from.y);
    double cosDistance = cos(distance);
    double sinDistance = sin(distance);
    double sinFromLat = sin(fromLat);
    double cosFromLat = cos(fromLat);
    double sinLat =
        cosDistance * sinFromLat + sinDistance * cosFromLat * cos(heading);
    double dLng = atan2(sinDistance * cosFromLat * sin(heading),
        cosDistance - sinFromLat * sinLat);
    return Point(toDegrees(asin(sinLat)), toDegrees(fromLng + dLng));
  }

  /// Returns the location of origin when provided with a Point destination,
  /// meters travelled and original heading. Headings are expressed in degrees
  /// clockwise from North. This function returns null when no solution is
  /// available.
  ///
  /// [to]       The destination Point.
  ///
  /// [distance] The distance travelled, in meters.
  ///
  /// [heading]  The heading in degrees clockwise from north.
  static Point? computeOffsetOrigin(Point to, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://lists.maptools.org/pipermail/proj/2008-October/003939.html
    double n1 = cos(distance);
    double n2 = sin(distance) * cos(heading);
    double n3 = sin(distance) * sin(heading);
    double n4 = sin(toRadians(to.x));
    // There are two solutions for b. b = n2 * n4 +/- sqrt(), one solution results
    // in the x outside the [-90, 90] range. We first try one solution and
    // back off to the other if we are outside that range.
    double n12 = n1 * n1;
    double discriminant = n2 * n2 * n12 + n12 * n12 - n12 * n4 * n4;

    // No real solution which would make sense in Point-space.
    if (discriminant < 0) {
      return null;
    }

    double b = n2 * n4 + sqrt(discriminant);
    b /= n1 * n1 + n2 * n2;
    double a = (n4 - n2 * b) / n1;
    double fromLatRadians = atan2(a, b);
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      b = n2 * n4 - sqrt(discriminant);
      b /= n1 * n1 + n2 * n2;
      fromLatRadians = atan2(a, b);
    }
    // No solution which would make sense in Point-space.
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      return null;
    }

    double fromLngRadians = toRadians(to.y) -
        atan2(n3, n1 * cos(fromLatRadians) - n2 * sin(fromLatRadians));
    return Point(toDegrees(fromLatRadians), toDegrees(fromLngRadians));
  }

  /// Returns the Point which lies the given fraction of the way between the
  /// origin Point and the destination Point.
  ///
  /// [from]     The Point from which to start.
  ///
  /// [to]       The Point toward which to travel.
  ///
  /// [fraction] A fraction of the distance to travel.
  ///
  /// [return] The interpolated Point.
  static Point interpolate(Point from, Point to, double fraction) {
    // http://en.wikipedia.org/wiki/Slerp
    double fromLat = toRadians(from.x);
    double fromLng = toRadians(from.y);
    double toLat = toRadians(to.x);
    double toLng = toRadians(to.y);
    double cosFromLat = cos(fromLat);
    double cosToLat = cos(toLat);

    // Computes Spherical interpolation coefficients.
    double angle = computeAngleBetween(from, to);
    double sinAngle = sin(angle);
    if (sinAngle < 1E-6) {
      return Point(from.x + fraction * (to.x - from.x),
          from.y + fraction * (to.y - from.y));
    }

    double a = sin((1 - fraction) * angle) / sinAngle;
    double b = sin(fraction * angle) / sinAngle;

    // Converts from polar to vector and interpolate.
    double x = a * cosFromLat * cos(fromLng) + b * cosToLat * cos(toLng);
    double y = a * cosFromLat * sin(fromLng) + b * cosToLat * sin(toLng);
    double z = a * sin(fromLat) + b * sin(toLat);

    // Converts interpolated vector back to polar.
    double lat = atan2(z, sqrt(x * x + y * y));
    double lng = atan2(y, x);
    return Point(toDegrees(lat), toDegrees(lng));
  }

  /// Returns distance on the unit sphere; the arguments are in radians.
  static double distanceRadians(
          double lat1, double lng1, double lat2, double lng2) =>
      MathUtils.arcHav(MathUtils.havDistance(lat1, lat2, lng1 - lng2));

  /// Returns the angle between two LatLngs, in radians. This is the same as the distance
  /// on the unit sphere.
  static double computeAngleBetween(Point from, Point to) => distanceRadians(
      toRadians(from.x), toRadians(from.y), toRadians(to.x), toRadians(to.y));

  /// Returns the distance between two LatLngs, in meters.
  static double computeDistanceBetween(Point from, Point to) =>
      computeAngleBetween(from, to) * MathUtils.earthRadius;

  /// Returns the length of the given path, in meters, on Earth.
  static double computeLength(List<Point> path) {
    if (path.length < 2) {
      return 0;
    }

    double length = 0;
    Point prev = path[0];
    double prevLat = toRadians(prev.x);
    double prevLng = toRadians(prev.y);
    for (final point in path) {
      double lat = toRadians(point.x);
      double lng = toRadians(point.y);
      length += distanceRadians(prevLat, prevLng, lat, lng);
      prevLat = lat;
      prevLng = lng;
    }
    return length * MathUtils.earthRadius;
  }

  /// Returns the area of a closed path on Earth.
  ///
  ///  [path] A closed path.
  ///
  ///  [return] The path's area in square meters.
  static double computeArea(List<Point> path) => computeSignedArea(path).abs();

  /// Returns the signed area of a closed path on Earth. The sign of the area may be used to
  /// determine the orientation of the path.
  /// 'inside' is the surface that does not contain the South Pole.
  ///
  /// [path] A closed path.
  ///
  /// [return] The loop's area in square meters.
  static double computeSignedArea(List<Point> path) =>
      SphericalUtils.computeSignedAreaTest(path, MathUtils.earthRadius);

  /// Returns the signed area of a closed path on a sphere of given radius.
  /// The computed area uses the same units as the radius squared.
  /// Used by SphericalUtilTest.
  static double computeSignedAreaTest(List<Point> path, double radius) {
    int size = path.length;
    if (size < 3) {
      return 0;
    }

    double total = 0;
    Point prev = path[size - 1];
    double prevTanLat = tan((pi / 2 - toRadians(prev.x)) / 2);
    double prevLng = toRadians(prev.y);
    // For each edge, accumulate the signed area of the triangle formed by the North Pole
    // and that edge ('polar triangle').
    for (final point in path) {
      double tanLat = tan((pi / 2 - toRadians(point.x)) / 2);
      double lng = toRadians(point.y);
      total += polarTriangleArea(tanLat, lng, prevTanLat, prevLng);
      prevTanLat = tanLat;
      prevLng = lng;
    }
    return total * (radius * radius);
  }

  /// Returns the signed area of a triangle which has North Pole as a vertex.
  /// Formula derived from 'Area of a spherical triangle given two edges and the included angle'
  /// as per 'Spherical Trigonometry' by Todhunter, page 71, section 103, point 2.
  ///
  /// See http://books.google.com/books?id=3uBHAAAAIAAJ&pg=PA71
  ///
  /// The arguments named 'tan' are tan((pi/2 - x)/2).
  static double polarTriangleArea(
      double tan1, double lng1, double tan2, double lng2) {
    double deltaLng = lng1 - lng2;
    double t = tan1 * tan2;
    return 2 * atan2(t * sin(deltaLng), 1 + t * cos(deltaLng));
  }
}
