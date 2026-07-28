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

/// Axis-aligned latitude/longitude bounding box.
class GoogleMapsUtilsLatLngBounds {
  /// The north-east corner of the bounding box.
  final Point<double> northEast;

  /// The south-west corner of the bounding box.
  final Point<double> southWest;

  /// Creates a bounding box from [northEast] and [southWest] corners.
  GoogleMapsUtilsLatLngBounds(
      {required this.northEast, required this.southWest});

  @override
  String toString() =>
      'GoogleMapsUtilsLatLngBounds(ne: (${northEast.x}, ${northEast.y}), sw: (${southWest.x}, ${southWest.y}))';

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is GoogleMapsUtilsLatLngBounds &&
          northEast == other.northEast &&
          southWest == other.southWest;

  @override
  int get hashCode => Object.hash(northEast, southWest);
}

/// Spherical geometry utilities (distances, headings, areas, offsets,
/// interpolation), ported from the android-maps-utils `SphericalUtil` class.
class SphericalUtils {
  SphericalUtils._();

  /// Converts [deg] degrees to radians.
  static double toRadians(double deg) => deg * (pi / 180.0);

  /// Converts [rad] radians to degrees.
  static double toDegrees(double rad) => rad * (180.0 / pi);

  /// Returns a 16-point compass cardinal abbreviation for the given [angle].
  ///
  /// See: https://stackoverflow.com/a/25867068/3182210
  static String getCardinal(double angle) {
    final val = ((angle / 22.5) + 0.5).floor();
    const arr = [
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

  /// Creates a bounding box centered at ([x], [y]) with a given
  /// [radiusInMeters].
  ///
  /// See: https://stackoverflow.com/a/31029389/3182210
  static GoogleMapsUtilsLatLngBounds toBounds(
      double x, double y, double radiusInMeters) {
    final center = Point<double>(x, y);
    final distanceFromCenterToCorner = radiusInMeters * sqrt(2.0);
    final southwestCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 225.0);
    final northeastCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 45.0);
    return GoogleMapsUtilsLatLngBounds(
      northEast: northeastCorner,
      southWest: southwestCorner,
    );
  }

  /// Computes the smallest bounding box that contains all [points].
  ///
  /// Throws if [points] is empty.
  ///
  /// See: https://github.com/flutter/flutter/issues/36653#issuecomment-525288053
  static GoogleMapsUtilsLatLngBounds toBoundsFromPoints(
      List<Point<double>> points) {
    if (points.isEmpty) throw ArgumentError('Points cannot be empty');

    double x0 = points.first.x;
    double x1 = points.first.x;
    double y0 = points.first.y;
    double y1 = points.first.y;

    for (final point in points) {
      if (point.x > x1) x1 = point.x;
      if (point.x < x0) x0 = point.x;
      if (point.y > y1) y1 = point.y;
      if (point.y < y0) y0 = point.y;
    }

    return GoogleMapsUtilsLatLngBounds(
      northEast: Point<double>(x1, y1),
      southWest: Point<double>(x0, y0),
    );
  }

  /// Returns the center point of the given [gmuLatLngBounds].
  ///
  /// See: https://stackoverflow.com/a/30859321/3182210
  static Point<double> centerFromLatLngBounds(
      GoogleMapsUtilsLatLngBounds gmuLatLngBounds) {
    final northEast = gmuLatLngBounds.northEast;
    final southWest = gmuLatLngBounds.southWest;

    final centerLat = (northEast.x + southWest.x) / 2;
    final centerLng = (northEast.y + southWest.y) / 2;

    return Point<double>(centerLat, centerLng);
  }

  /// Returns the heading from [from] to [to]. Headings are expressed in degrees
  /// clockwise from North within the range [-180, 180).
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

  /// Returns the point resulting from moving [distance] meters from [from] in
  /// the specified [heading] (degrees clockwise from north).
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

  /// Returns the origin when provided with a destination [to], [distance] in
  /// meters, and original [heading]. Returns `null` when no solution exists.
  static Point<double>? computeOffsetOrigin(
      Point<double> to, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://lists.maptools.org/pipermail/proj/2008-October/003939.html
    final n1 = cos(distance);
    final n2 = sin(distance) * cos(heading);
    final n3 = sin(distance) * sin(heading);
    final n4 = sin(toRadians(to.x));
    // There are two solutions for b. b = n2 * n4 +/- sqrt(), one solution
    // results in the latitude outside the [-90, 90] range. We first try one
    // solution and back off to the other if we are outside that range.
    final n12 = n1 * n1;
    final discriminant = n2 * n2 * n12 + n12 * n12 - n12 * n4 * n4;

    // No real solution which would make sense in lat-lng space.
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
    // No solution which would make sense in lat-lng space.
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      return null;
    }

    final fromLngRadians = toRadians(to.y) -
        atan2(n3, n1 * cos(fromLatRadians) - n2 * sin(fromLatRadians));
    return Point(toDegrees(fromLatRadians), toDegrees(fromLngRadians));
  }

  /// Returns the point which lies the given [fraction] of the way between
  /// [from] and [to].
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

  /// Returns the angle between [from] and [to], in radians. This is the same
  /// as the distance on the unit sphere.
  static double computeAngleBetween(Point<double> from, Point<double> to) =>
      distanceRadians(toRadians(from.x), toRadians(from.y), toRadians(to.x),
          toRadians(to.y));

  /// Returns the distance between [from] and [to], in meters.
  static double computeDistanceBetween(Point<double> from, Point<double> to) =>
      computeAngleBetween(from, to) * MathUtils.earthRadius;

  /// Returns the length of the given [path], in meters, on Earth.
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

  /// Returns the area of a closed [path] on Earth, in square meters.
  static double computeArea(List<Point<double>> path) =>
      computeSignedArea(path).abs();

  /// Returns the signed area of a closed [path] on Earth. The sign of the area
  /// may be used to determine the orientation of the path. "Inside" is the
  /// surface that does not contain the South Pole.
  ///
  /// Returns the area in square meters.
  static double computeSignedArea(List<Point<double>> path) =>
      _computeSignedAreaWithRadius(path, MathUtils.earthRadius);

  /// Returns the signed area of a closed [path] on a sphere of given [radius].
  /// The computed area uses the same units as the radius squared.
  ///
  /// This is exposed for testing; prefer [computeSignedArea] for production use.
  static double computeSignedAreaTest(
          List<Point<double>> path, double radius) =>
      _computeSignedAreaWithRadius(path, radius);

  static double _computeSignedAreaWithRadius(
      List<Point<double>> path, double radius) {
    final size = path.length;
    if (size < 3) {
      return 0;
    }

    double total = 0;
    final prev = path[size - 1];
    double prevTanLat = tan((pi / 2 - toRadians(prev.x)) / 2);
    double prevLng = toRadians(prev.y);
    // For each edge, accumulate the signed area of the triangle formed by the
    // North Pole and that edge ('polar triangle').
    for (final point in path) {
      final tanLat = tan((pi / 2 - toRadians(point.x)) / 2);
      final lng = toRadians(point.y);
      total += _polarTriangleArea(tanLat, lng, prevTanLat, prevLng);
      prevTanLat = tanLat;
      prevLng = lng;
    }
    return total * (radius * radius);
  }

  /// Returns the signed area of a triangle which has North Pole as a vertex.
  /// Formula derived from 'Area of a spherical triangle given two edges and the
  /// included angle' as per 'Spherical Trigonometry' by Todhunter, page 71,
  /// section 103, point 2.
  ///
  /// See http://books.google.com/books?id=3uBHAAAAIAAJ&pg=PA71
  ///
  /// The arguments named 'tan' are tan((pi/2 - lat)/2).
  static double _polarTriangleArea(
      double tan1, double lng1, double tan2, double lng2) {
    final deltaLng = lng1 - lng2;
    final t = tan1 * tan2;
    return 2 * atan2(t * sin(deltaLng), 1 + t * cos(deltaLng));
  }

  /// Splits [bounds] into smaller sub-bounds. The number of resulting bounds
  /// equals [division]².
  static List<GoogleMapsUtilsLatLngBounds> toSubBounds(
    GoogleMapsUtilsLatLngBounds bounds, {
    int division = 2,
  }) {
    final subBounds = <GoogleMapsUtilsLatLngBounds>[];

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
          GoogleMapsUtilsLatLngBounds(
              northEast: newNorthEast, southWest: newSouthWest),
        );
      }
    }

    return subBounds;
  }
}
