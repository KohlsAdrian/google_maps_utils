//  Copyright 2013 Google Inc.
//
//  Licensed under the Apache License, Version 2.0 (the 'License');
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an 'AS IS' BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';

class GMULatLngBounds {
  final Point<double> northEast;
  final Point<double> southWest;

  GMULatLngBounds({required this.northEast, required this.southWest});
}

class SphericalUtils {
  SphericalUtils._();

  ///  Missing simple conversions from Math class
  ///  Code from: https://github.com/dart-lang/sdk/issues/4211#issue-84512743
  static double toRadians(double deg) => deg * (pi / 180.0);
  static double toDegrees(double rad) => rad * (180.0 / pi);

  /// see: https://stackoverflow.com/a/25867068/3182210
  static String getCardinal(double angle) {
    final val = ((angle / 22.5) + 0.5).floor();
    final arr = [
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
    final center = Point<double>(x, y);
    final distanceFromCenterToCorner = radiusInMeters * sqrt(2.0);
    final southwestCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 225.0);
    final northeastCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 45.0);
    return GMULatLngBounds(
      northEast: northeastCorner,
      southWest: southwestCorner,
    );
  }

  /// Convert a list of latitudes and longitudes to an object of boundaries
  ///
  /// see: https://github.com/flutter/flutter/issues/36653#issuecomment-525288053
  ///
  /// Returns The boundaries from the list of points
  static GMULatLngBounds toBoundsFromPoints(List<Point<double>> points) {
    if (points.isEmpty) throw Exception("Points cannot be empty");

    double x0, x1, y0, y1;
    x0 = x1 = points.first.x;
    y0 = y1 = points.first.y;
    points.forEach((point) {
      if (point.x > x1) x1 = point.x;
      if (point.x < x0) x0 = point.x;
      if (point.y > y1) y1 = point.y;
      if (point.y < y0) y0 = point.y;
    });

    final northEast = Point<double>(x1, y1);
    final southWest = Point<double>(x0, y0);

    final gmuLatLngBounds = GMULatLngBounds(
      northEast: northEast,
      southWest: southWest,
    );

    return gmuLatLngBounds;
  }

  /// Calculate the center of the boundaries to a Point in latitude and longitude
  ///
  /// see: https://stackoverflow.com/a/30859321/3182210
  ///
  /// Returns the Point representing the center of the [gmuLatLngBounds]
  static Point<double> centerFromLatLngBounds(GMULatLngBounds gmuLatLngBounds) {
    final northEast = gmuLatLngBounds.northEast;
    final southWest = gmuLatLngBounds.southWest;

    final centerLatitudeBound = (northEast.x + southWest.x) / 2;
    final centerLongitudeBound = (northEast.y + southWest.y) / 2;

    final center = Point<double>(centerLatitudeBound, centerLongitudeBound);

    return center;
  }

  /// Returns the heading from one Point to another Point. Headings are
  /// expressed in degrees clockwise from North within the range [-180,180).
  ///
  /// Returns The heading in degrees clockwise from north.
  static double computeHeading(Point<double> from, Point<double> to) {
    // http://williams.best.vwh.net/avform.htm#Crs

    final fromLat = toRadians(from.x);
    final fromLng = toRadians(from.y);
    final toLat = toRadians(to.x);
    final toLng = toRadians(to.y);
    final dLng = toLng - fromLng;
    final heading = atan2(sin(dLng) * cos(toLat),
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
  static Point<double> computeOffset(
      Point<double> from, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://williams.best.vwh.net/avform.htm#LL
    final fromLat = toRadians(from.x);
    final fromLng = toRadians(from.y);
    final cosDistance = cos(distance);
    final sinDistance = sin(distance);
    final sinFromLat = sin(fromLat);
    final cosFromLat = cos(fromLat);
    final sinLat =
        cosDistance * sinFromLat + sinDistance * cosFromLat * cos(heading);
    final dLng = atan2(sinDistance * cosFromLat * sin(heading),
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
  static Point<double>? computeOffsetOrigin(
      Point<double> to, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://lists.maptools.org/pipermail/proj/2008-October/003939.html
    final n1 = cos(distance);
    final n2 = sin(distance) * cos(heading);
    final n3 = sin(distance) * sin(heading);
    final n4 = sin(toRadians(to.x));
    // There are two solutions for b. b = n2 * n4 +/- sqrt(), one solution results
    // in the x outside the [-90, 90] range. We first try one solution and
    // back off to the other if we are outside that range.
    final n12 = n1 * n1;
    final discriminant = n2 * n2 * n12 + n12 * n12 - n12 * n4 * n4;

    // No real solution which would make sense in Point<double>-space.
    if (discriminant < 0) {
      return null;
    }

    double b = n2 * n4 + sqrt(discriminant);
    b /= n1 * n1 + n2 * n2;
    final a = (n4 - n2 * b) / n1;
    double fromLatRadians = atan2(a, b);
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      b = n2 * n4 - sqrt(discriminant);
      b /= n1 * n1 + n2 * n2;
      fromLatRadians = atan2(a, b);
    }
    // No solution which would make sense in Point<double>-space.
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      return null;
    }

    final fromLngRadians = toRadians(to.y) -
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
  /// Returns The interpolated Point.
  static Point<double> interpolate(
      Point<double> from, Point<double> to, double fraction) {
    // http://en.wikipedia.org/wiki/Slerp
    final fromLat = toRadians(from.x);
    final fromLng = toRadians(from.y);
    final toLat = toRadians(to.x);
    final toLng = toRadians(to.y);
    final cosFromLat = cos(fromLat);
    final cosToLat = cos(toLat);

    // Computes Spherical interpolation coefficients.
    final angle = computeAngleBetween(from, to);
    final sinAngle = sin(angle);
    if (sinAngle < 1E-6) {
      return Point(from.x + fraction * (to.x - from.x),
          from.y + fraction * (to.y - from.y));
    }

    final a = sin((1 - fraction) * angle) / sinAngle;
    final b = sin(fraction * angle) / sinAngle;

    // Converts from polar to vector and interpolate.
    final x = a * cosFromLat * cos(fromLng) + b * cosToLat * cos(toLng);
    final y = a * cosFromLat * sin(fromLng) + b * cosToLat * sin(toLng);
    final z = a * sin(fromLat) + b * sin(toLat);

    // Converts interpolated vector back to polar.
    final lat = atan2(z, sqrt(x * x + y * y));
    final lng = atan2(y, x);
    return Point(toDegrees(lat), toDegrees(lng));
  }

  /// Returns distance on the unit sphere; the arguments are in radians.
  static double distanceRadians(
          double lat1, double lng1, double lat2, double lng2) =>
      MathUtils.arcHav(MathUtils.havDistance(lat1, lat2, lng1 - lng2));

  /// Returns the angle between two LatLngs, in radians. This is the same as the distance
  /// on the unit sphere.
  static double computeAngleBetween(Point<double> from, Point<double> to) =>
      distanceRadians(toRadians(from.x), toRadians(from.y), toRadians(to.x),
          toRadians(to.y));

  /// Returns the distance between two LatLngs, in meters.
  static double computeDistanceBetween(Point<double> from, Point<double> to) =>
      computeAngleBetween(from, to) * MathUtils.earthRadius;

  /// Returns the distance summed from all LatLng points, in meters
  static double computeDistanceFromListOfPoints(List<Point<double>> points) {
    double total = 0.0;

    for (int i = 0; i < points.length - 1; i++) {
      total += computeDistanceBetween(points[i], points[i + 1]);
    }

    return total;
  }

  /// Returns the length of the given path, in meters, on Earth.
  static double computeLength(List<Point<double>> path) {
    if (path.length < 2) {
      return 0;
    }

    double length = 0;
    final prev = path[0];
    double prevLat = toRadians(prev.x);
    double prevLng = toRadians(prev.y);
    for (final point in path) {
      final lat = toRadians(point.x);
      final lng = toRadians(point.y);
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
  ///  Returns The path's area in square meters.
  static double computeArea(List<Point<double>> path) =>
      computeSignedArea(path).abs();

  /// Returns the signed area of a closed path on Earth. The sign of the area may be used to
  /// determine the orientation of the path.
  /// 'inside' is the surface that does not contain the South Pole.
  ///
  /// [path] A closed path.
  ///
  /// Returns The loop's area in square meters.
  static double computeSignedArea(List<Point<double>> path) =>
      SphericalUtils.computeSignedAreaTest(path, MathUtils.earthRadius);

  /// Returns the signed area of a closed path on a sphere of given radius.
  /// The computed area uses the same units as the radius squared.
  /// Used by SphericalUtilTest.
  static double computeSignedAreaTest(List<Point<double>> path, double radius) {
    final size = path.length;
    if (size < 3) {
      return 0;
    }

    double total = 0;
    final prev = path[size - 1];
    double prevTanLat = tan((pi / 2 - toRadians(prev.x)) / 2);
    double prevLng = toRadians(prev.y);
    // For each edge, accumulate the signed area of the triangle formed by the North Pole
    // and that edge ('polar triangle').
    for (final point in path) {
      final tanLat = tan((pi / 2 - toRadians(point.x)) / 2);
      final lng = toRadians(point.y);
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
    final deltaLng = lng1 - lng2;
    final t = tan1 * tan2;
    return 2 * atan2(t * sin(deltaLng), 1 + t * cos(deltaLng));
  }

  /// Splits Bound to smaller bounds equivalent of number of `division`^2
  static List<GMULatLngBounds> toSubBounds(
    GMULatLngBounds bounds, {
    int division = 2,
  }) {
    final subBounds = <GMULatLngBounds>[];

    final northEast = bounds.northEast;
    final southWest = bounds.southWest;

    final xDistanceStep = (northEast.x - southWest.x) / division;
    final yDistanceStep = (northEast.y - southWest.y) / division;

    for (int i = 1; i <= division; i++) {
      final currentX = southWest.x;
      final currentY = southWest.y;
      for (int j = 1; j <= division; j++) {
        final newNorthEast = Point(
          currentX + xDistanceStep * i,
          currentY + yDistanceStep * j,
        );
        final newSouthWest = Point(
          currentX + (xDistanceStep * (i - 1)),
          currentY + (yDistanceStep * (j - 1)),
        );

        subBounds.add(
          GMULatLngBounds(northEast: newNorthEast, southWest: newSouthWest),
        );
      }
    }

    return subBounds;
  }
}
