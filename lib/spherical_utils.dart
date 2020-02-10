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

class SphericalUtils {
  ///  Missing simple conversions from Math class
  ///  Code from: https://github.com/dart-lang/sdk/issues/4211#issue-84512743
  static num toRadians(num deg) => deg * (pi / 180.0);
  static num toDegrees(num rad) => rad * (180.0 / pi);

  /// see: https://stackoverflow.com/a/25867068/3182210
  static String getCardinal(double angle) {
    var val = ((angle / 22.5) + 0.5).floor();
    var arr = [
      'N',
      //'NNE',
      'NE',
      //'ENE',
      'E',
      //'ESE',
      'SE',
      //'SSE',
      'S',
      //'SSW',
      'SW',
      //'WSW',
      'W',
      //'WNW',
      'NW',
      //'NNW'
    ];
    //16
    return arr[(val % 8)];
  }

  /// Get Translation from North-American Direction to PT-BR direction
  static String getDirectionName(String direction) {
    //4 basic direction
    if (direction == 'N') {
      direction = 'Norte';
    } else if (direction == 'S') {
      direction = 'Sul';
    } else if (direction == 'E') {
      direction = 'Leste';
    } else if (direction == 'W') {
      direction = 'Oeste';
    }
    //4+ direction
    else if (direction == 'NE') {
      direction = 'Nordeste';
    } else if (direction == 'SE') {
      direction = 'Sudeste';
    } else if (direction == 'SW') {
      direction = 'Sudoeste';
    } else if (direction == 'NW') {
      direction = 'Noroeste';
    }
    return direction;
  }

  /// see: https://stackoverflow.com/a/31029389/3182210
  static LatLngBounds toBounds(
      double latitude, double longitude, double radiusInMeters) {
    LatLng center = LatLng(latitude, longitude);
    double distanceFromCenterToCorner = radiusInMeters * sqrt(2.0);
    LatLng southwestCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 225.0);
    LatLng northeastCorner =
        SphericalUtils.computeOffset(center, distanceFromCenterToCorner, 45.0);
    return LatLngBounds(southwestCorner, northeastCorner);
  }

  /// Returns the heading from one LatLng to another LatLng. Headings are
  /// expressed in degrees clockwise from North within the range [-180,180).
  ///
  /// [return] The heading in degrees clockwise from north.
  static double computeHeading(LatLng from, LatLng to) {
    // http://williams.best.vwh.net/avform.htm#Crs

    double fromLat = toRadians(from.latitude);
    double fromLng = toRadians(from.longitude);
    double toLat = toRadians(to.latitude);
    double toLng = toRadians(to.longitude);
    double dLng = toLng - fromLng;
    double heading = atan2(sin(dLng) * cos(toLat),
        cos(fromLat) * sin(toLat) - sin(fromLat) * cos(toLat) * cos(dLng));
    return MathUtils.wrap(toDegrees(heading), -180, 180);
  }

  /// Returns the LatLng resulting from moving a distance from an origin
  /// in the specified heading (expressed in degrees clockwise from north).
  ///
  /// [from]     The LatLng from which to start.
  /// [distance] The distance to travel.
  /// [heading]  The heading in degrees clockwise from north.
  static LatLng computeOffset(LatLng from, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://williams.best.vwh.net/avform.htm#LL
    double fromLat = toRadians(from.latitude);
    double fromLng = toRadians(from.longitude);
    double cosDistance = cos(distance);
    double sinDistance = sin(distance);
    double sinFromLat = sin(fromLat);
    double cosFromLat = cos(fromLat);
    double sinLat =
        cosDistance * sinFromLat + sinDistance * cosFromLat * cos(heading);
    double dLng = atan2(sinDistance * cosFromLat * sin(heading),
        cosDistance - sinFromLat * sinLat);
    return LatLng(toDegrees(asin(sinLat)), toDegrees(fromLng + dLng));
  }

  /// Returns the location of origin when provided with a LatLng destination,
  /// meters travelled and original heading. Headings are expressed in degrees
  /// clockwise from North. This function returns null when no solution is
  /// available.
  ///
  /// [to]       The destination LatLng.
  /// [distance] The distance travelled, in meters.
  /// [heading]  The heading in degrees clockwise from north.
  static LatLng computeOffsetOrigin(
      LatLng to, double distance, double heading) {
    distance /= MathUtils.earthRadius;
    heading = toRadians(heading);
    // http://lists.maptools.org/pipermail/proj/2008-October/003939.html
    double n1 = cos(distance);
    double n2 = sin(distance) * cos(heading);
    double n3 = sin(distance) * sin(heading);
    double n4 = sin(toRadians(to.latitude));
    // There are two solutions for b. b = n2 * n4 +/- sqrt(), one solution results
    // in the latitude outside the [-90, 90] range. We first try one solution and
    // back off to the other if we are outside that range.
    double n12 = n1 * n1;
    double discriminant = n2 * n2 * n12 + n12 * n12 - n12 * n4 * n4;

    // No real solution which would make sense in LatLng-space.
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
    // No solution which would make sense in LatLng-space.
    if (fromLatRadians < -pi / 2 || fromLatRadians > pi / 2) {
      return null;
    }

    double fromLngRadians = toRadians(to.longitude) -
        atan2(n3, n1 * cos(fromLatRadians) - n2 * sin(fromLatRadians));
    return LatLng(toDegrees(fromLatRadians), toDegrees(fromLngRadians));
  }

  /// Returns the LatLng which lies the given fraction of the way between the
  /// origin LatLng and the destination LatLng.
  ///
  /// [from]     The LatLng from which to start.
  /// [to]       The LatLng toward which to travel.
  /// [fraction] A fraction of the distance to travel.
  /// [return] The interpolated LatLng.
  static LatLng interpolate(LatLng from, LatLng to, double fraction) {
    // http://en.wikipedia.org/wiki/Slerp
    double fromLat = toRadians(from.latitude);
    double fromLng = toRadians(from.longitude);
    double toLat = toRadians(to.latitude);
    double toLng = toRadians(to.longitude);
    double cosFromLat = cos(fromLat);
    double cosToLat = cos(toLat);

    // Computes Spherical interpolation coefficients.
    double angle = computeAngleBetween(from, to);
    double sinAngle = sin(angle);
    if (sinAngle < 1E-6) {
      return LatLng(from.latitude + fraction * (to.latitude - from.latitude),
          from.longitude + fraction * (to.longitude - from.longitude));
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
    return LatLng(toDegrees(lat), toDegrees(lng));
  }

  /// Returns distance on the unit sphere; the arguments are in radians.
  static double distanceRadians(
          double lat1, double lng1, double lat2, double lng2) =>
      MathUtils.arcHav(MathUtils.havDistance(lat1, lat2, lng1 - lng2));

  /// Returns the angle between two LatLngs, in radians. This is the same as the distance
  /// on the unit sphere.
  static double computeAngleBetween(LatLng from, LatLng to) => distanceRadians(
      toRadians(from.latitude),
      toRadians(from.longitude),
      toRadians(to.latitude),
      toRadians(to.longitude));

  /// Returns the distance between two LatLngs, in meters.
  static double computeDistanceBetween(LatLng from, LatLng to) =>
      computeAngleBetween(from, to) * MathUtils.earthRadius;

  /// Returns the length of the given path, in meters, on Earth.
  static double computeLength(List<LatLng> path) {
    if (path.length < 2) {
      return 0;
    }

    double length = 0;
    LatLng prev = path[0];
    double prevLat = toRadians(prev.latitude);
    double prevLng = toRadians(prev.longitude);
    for (final point in path) {
      double lat = toRadians(point.latitude);
      double lng = toRadians(point.longitude);
      length += distanceRadians(prevLat, prevLng, lat, lng);
      prevLat = lat;
      prevLng = lng;
    }
    return length * MathUtils.earthRadius;
  }

  /// Returns the area of a closed path on Earth.
  ///  [path] A closed path.
  ///  [return] The path's area in square meters.
  static double computeArea(List<LatLng> path) => computeSignedArea(path).abs();

  /// Returns the signed area of a closed path on Earth. The sign of the area may be used to
  /// determine the orientation of the path.
  /// 'inside' is the surface that does not contain the South Pole.
  ///
  /// [path] A closed path.
  /// [return] The loop's area in square meters.
  static double computeSignedArea(List<LatLng> path) =>
      SphericalUtils.computeSignedAreaTest(path, MathUtils.earthRadius);

  /// Returns the signed area of a closed path on a sphere of given radius.
  /// The computed area uses the same units as the radius squared.
  /// Used by SphericalUtilTest.
  static double computeSignedAreaTest(List<LatLng> path, double radius) {
    int size = path.length;
    if (size < 3) {
      return 0;
    }

    double total = 0;
    LatLng prev = path[size - 1];
    double prevTanLat = tan((pi / 2 - toRadians(prev.latitude)) / 2);
    double prevLng = toRadians(prev.longitude);
    // For each edge, accumulate the signed area of the triangle formed by the North Pole
    // and that edge ('polar triangle').
    for (final point in path) {
      double tanLat = tan((pi / 2 - toRadians(point.latitude)) / 2);
      double lng = toRadians(point.longitude);
      total += polarTriangleArea(tanLat, lng, prevTanLat, prevLng);
      prevTanLat = tanLat;
      prevLng = lng;
    }
    return total * (radius * radius);
  }

  /// Returns the signed area of a triangle which has North Pole as a vertex.
  /// Formula derived from 'Area of a spherical triangle given two edges and the included angle'
  /// as per 'Spherical Trigonometry' by Todhunter, page 71, section 103, point 2.
  /// See http://books.google.com/books?id=3uBHAAAAIAAJ&pg=PA71
  /// The arguments named 'tan' are tan((pi/2 - latitude)/2).
  static double polarTriangleArea(
      double tan1, double lng1, double tan2, double lng2) {
    double deltaLng = lng1 - lng2;
    double t = tan1 * tan2;
    return 2 * atan2(t * sin(deltaLng), 1 + t * cos(deltaLng));
  }
}
