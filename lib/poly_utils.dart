import 'dart:math';

// Copyright 2008, 2013 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
import 'package:google_maps_utils/google_maps_utils.dart';

import 'utils/stack.dart';

/// Polyline and polygon utilities (point-in-polygon, on-path/on-edge tests,
/// Douglas-Peucker simplification, and polyline encoding/decoding), ported from
/// the android-maps-utils `PolyUtil` class.
class PolyUtils {
  PolyUtils._();

  /// Checks whether [point] is inside [polygon] on the sphere.
  ///
  /// A faithful port of android-maps-utils' `PolyUtil.containsLocation`. It casts
  /// a meridian ray from the point to a pole and counts edge crossings, so it is
  /// correct for polygons of any size — including ones that cross the
  /// antimeridian (±180° longitude) or enclose a pole. "Inside" is the region
  /// that does not contain the South Pole.
  ///
  /// Edges are treated as great-circle (geodesic) segments when [geodesic] is
  /// `true`, and as Rhumb (constant-bearing) segments otherwise. The default
  /// (`geodesic: false`) matches how polygons are usually drawn on a Mercator map
  /// and the planar behaviour of earlier versions for small areas.
  ///
  /// The polygon is implicitly closed, so the closing point may be omitted or
  /// repeated. A [point] coinciding with a vertex is considered inside; the
  /// result for a [point] lying exactly on an edge is not defined.
  ///
  /// Coordinates follow this package's convention: `Point.x` is latitude and
  /// `Point.y` is longitude, in degrees.
  static bool containsLocationPoly(
    Point<double> point,
    List<Point<double>> polygon, {
    bool geodesic = false,
  }) {
    final size = polygon.length;
    if (size == 0) return false;

    final lat3 = SphericalUtils.toRadians(point.x);
    final lng3 = SphericalUtils.toRadians(point.y);
    final prev = polygon[size - 1];
    double lat1 = SphericalUtils.toRadians(prev.x);
    double lng1 = SphericalUtils.toRadians(prev.y);
    int nIntersect = 0;
    for (final point2 in polygon) {
      final dLng3 = MathUtils.wrap(lng3 - lng1, -pi, pi);
      // Special case: point equal to a vertex is inside.
      if (lat3 == lat1 && dLng3 == 0) return true;
      final lat2 = SphericalUtils.toRadians(point2.x);
      final lng2 = SphericalUtils.toRadians(point2.y);
      // Offset longitudes by -lng1.
      if (_intersects(lat1, lat2, MathUtils.wrap(lng2 - lng1, -pi, pi), lat3,
          dLng3, geodesic)) {
        nIntersect++;
      }
      lat1 = lat2;
      lng1 = lng2;
    }
    return (nIntersect & 1) != 0;
  }

  /// Whether the meridian segment from (`lat3`, `lng3`) to the North Pole
  /// crosses the polygon edge from (`lat1`, 0) to (`lat2`, `lng2`). Longitudes
  /// have already been offset so the edge's first point sits at longitude 0.
  static bool _intersects(double lat1, double lat2, double lng2, double lat3,
      double lng3, bool geodesic) {
    // Both ends on the same side of lng3.
    if ((lng3 >= 0 && lng3 >= lng2) || (lng3 < 0 && lng3 < lng2)) return false;
    // Point is South Pole.
    if (lat3 <= -pi / 2) return false;
    // Any segment end is a pole.
    if (lat1 <= -pi / 2 ||
        lat2 <= -pi / 2 ||
        lat1 >= pi / 2 ||
        lat2 >= pi / 2) {
      return false;
    }
    if (lng2 <= -pi) return false;
    final linearLat = (lat1 * (lng2 - lng3) + lat2 * lng3) / lng2;
    // Northern hemisphere and point under lat-lng line.
    if (lat1 >= 0 && lat2 >= 0 && lat3 < linearLat) return false;
    // Southern hemisphere and point above lat-lng line.
    if (lat1 <= 0 && lat2 <= 0 && lat3 >= linearLat) return true;
    // North Pole.
    if (lat3 >= pi / 2) return true;
    // Compare lat3 with the latitude on the GC/Rhumb segment at lng3 through a
    // strictly-increasing function.
    return geodesic
        ? tan(lat3) >= _tanLatGC(lat1, lat2, lng2, lng3)
        : MathUtils.mercator(lat3) >= _mercatorLatRhumb(lat1, lat2, lng2, lng3);
  }

  static double _tanLatGC(double lat1, double lat2, double lng2, double lng3) =>
      (tan(lat1) * sin(lng2 - lng3) + tan(lat2) * sin(lng3)) / sin(lng2);

  static double _mercatorLatRhumb(
          double lat1, double lat2, double lng2, double lng3) =>
      (MathUtils.mercator(lat1) * (lng2 - lng3) +
          MathUtils.mercator(lat2) * lng3) /
      lng2;

  static const _defaultTolerance = 0.1; // meters

  /// Computes whether the given [point] lies on or near the edge of a [polygon],
  /// within a specified [tolerance] in meters. The polygon edge is composed of
  /// great circle segments if [geodesic] is true, and of Rhumb segments
  /// otherwise. The polygon edge is implicitly closed — the closing segment
  /// between the first point and the last point is included.
  static bool isLocationOnEdgeTolerance(Point<double> point,
          List<Point<double>> polygon, bool geodesic, double tolerance) =>
      _isLocationOnEdgeOrPath(point, polygon, true, geodesic, tolerance);

  /// Same as [isLocationOnEdgeTolerance] with a default tolerance of 0.1 meters.
  static bool isLocationOnEdge(
          Point<double> point, List<Point<double>> polygon, bool geodesic) =>
      isLocationOnEdgeTolerance(point, polygon, geodesic, _defaultTolerance);

  /// Computes whether the given [point] lies on or near a [polyline], within a
  /// specified [tolerance] in meters. The polyline is composed of great circle
  /// segments if [geodesic] is true, and of Rhumb segments otherwise. The
  /// polyline is not closed — the closing segment between the first point and
  /// the last point is not included.
  static bool isLocationOnPathTolerance(Point<double> point,
          List<Point<double>> polyline, bool geodesic, double tolerance) =>
      _isLocationOnEdgeOrPath(point, polyline, false, geodesic, tolerance);

  /// Same as [isLocationOnPathTolerance] with a default tolerance of 0.1 meters.
  static bool isLocationOnPath(
          Point<double> point, List<Point<double>> polyline, bool geodesic) =>
      isLocationOnPathTolerance(point, polyline, geodesic, _defaultTolerance);

  static bool _isLocationOnEdgeOrPath(
          Point<double> point,
          List<Point<double>> poly,
          bool closed,
          bool geodesic,
          double toleranceEarth) =>
      locationIndexOnEdgeOrPath(
          point, poly, closed, geodesic, toleranceEarth) >=
      0;

  /// Computes whether (and where) a given [point] lies on or near a polyline,
  /// within a specified [tolerance] in meters.
  ///
  /// The polyline is not closed — the closing segment between the first point
  /// and the last point is not included.
  ///
  /// Returns -1 if the point does not lie on or near the polyline.
  /// Returns 0 if the point is between `poly[0]` and `poly[1]` (inclusive),
  /// 1 if between `poly[1]` and `poly[2]`, etc.
  static int locationIndexOnPathTolerance(Point<double> point,
          List<Point<double>> poly, bool geodesic, double tolerance) =>
      locationIndexOnEdgeOrPath(point, poly, false, geodesic, tolerance);

  /// Same as [locationIndexOnPathTolerance] with a default tolerance of 0.1 meters.
  static int locationIndexOnPath(
          Point<double> point, List<Point<double>> polyline, bool geodesic) =>
      locationIndexOnPathTolerance(
          point, polyline, geodesic, _defaultTolerance);

  /// Computes whether (and where) a given [point] lies on or near a polyline,
  /// within a specified [toleranceEarth] in meters.
  ///
  /// If [closed], the closing segment between the last and first points of the
  /// polyline is considered.
  ///
  /// Returns -1 if the point does not lie on or near the polyline.
  /// Returns 0 if the point is between `poly[0]` and `poly[1]` (inclusive),
  /// 1 if between `poly[1]` and `poly[2]`, etc.
  static int locationIndexOnEdgeOrPath(
      Point<double> point,
      List<Point<double>> poly,
      bool closed,
      bool geodesic,
      double toleranceEarth) {
    final size = poly.length;
    if (size == 0) {
      return -1;
    }

    final tolerance = toleranceEarth / MathUtils.earthRadius;
    final havTolerance = MathUtils.hav(tolerance);
    final lat3 = SphericalUtils.toRadians(point.x);
    final lng3 = SphericalUtils.toRadians(point.y);
    final prev = poly[closed ? size - 1 : 0];
    double lat1 = SphericalUtils.toRadians(prev.x);
    double lng1 = SphericalUtils.toRadians(prev.y);
    int idx = 0;
    if (geodesic) {
      for (final point2 in poly) {
        final lat2 = SphericalUtils.toRadians(point2.x);
        final lng2 = SphericalUtils.toRadians(point2.y);
        if (_isOnSegmentGC(lat1, lng1, lat2, lng2, lat3, lng3, havTolerance)) {
          return max(0, idx - 1);
        }
        lat1 = lat2;
        lng1 = lng2;
        idx++;
      }
    } else {
      // We project the points to mercator space, where the Rhumb segment is a
      // straight line, and compute the geodesic distance between point3 and the
      // closest point on the segment. This method is an approximation, because
      // it uses "closest" in mercator space which is not "closest" on the
      // sphere — but the error is small because "tolerance" is small.
      final minAcceptable = lat3 - tolerance;
      final maxAcceptable = lat3 + tolerance;
      double y1 = MathUtils.mercator(lat1);
      final y3 = MathUtils.mercator(lat3);
      final xTry = List<double>.filled(3, 0.0);
      for (final point2 in poly) {
        final lat2 = SphericalUtils.toRadians(point2.x);
        final y2 = MathUtils.mercator(lat2);
        final lng2 = SphericalUtils.toRadians(point2.y);
        if (max(lat1, lat2) >= minAcceptable &&
            min(lat1, lat2) <= maxAcceptable) {
          // We offset ys by -lng1; the implicit x1 is 0.
          final x2 = MathUtils.wrap(lng2 - lng1, -pi, pi);
          final x3Base = MathUtils.wrap(lng3 - lng1, -pi, pi);
          xTry[0] = x3Base;
          // Also explore wrapping of x3Base around the world in both directions.
          xTry[1] = x3Base + 2 * pi;
          xTry[2] = x3Base - 2 * pi;
          for (final x3 in xTry) {
            final dy = y2 - y1;
            final len2 = x2 * x2 + dy * dy;
            final t = len2 <= 0
                ? 0.0
                : MathUtils.clamp((x3 * x2 + (y3 - y1) * dy) / len2, 0, 1);
            final xClosest = t * x2;
            final yClosest = y1 + t * dy;
            final latClosest = MathUtils.inverseMercator(yClosest);
            final havDist =
                MathUtils.havDistance(lat3, latClosest, x3 - xClosest);

            if (havDist < havTolerance) return max(0, idx - 1);
          }
        }
        lat1 = lat2;
        lng1 = lng2;
        y1 = y2;
        idx++;
      }
    }
    return -1;
  }

  /// Returns sin(initial bearing from (lat1,lng1) to (lat3,lng3) minus initial
  /// bearing from (lat1, lng1) to (lat2,lng2)).
  static double _sinDeltaBearing(double lat1, double lng1, double lat2,
      double lng2, double lat3, double lng3) {
    final sinLat1 = sin(lat1);
    final cosLat2 = cos(lat2);
    final cosLat3 = cos(lat3);
    final lat31 = lat3 - lat1;
    final lng31 = lng3 - lng1;
    final lat21 = lat2 - lat1;
    final lng21 = lng2 - lng1;
    final a = sin(lng31) * cosLat3;
    final c = sin(lng21) * cosLat2;
    final b = sin(lat31) + 2 * sinLat1 * cosLat3 * MathUtils.hav(lng31);
    final d = sin(lat21) + 2 * sinLat1 * cosLat2 * MathUtils.hav(lng21);
    final denom = (a * a + b * b) * (c * c + d * d);
    return denom <= 0 ? 1 : (a * d - b * c) / sqrt(denom);
  }

  static bool _isOnSegmentGC(double lat1, double lng1, double lat2, double lng2,
      double lat3, double lng3, double havTolerance) {
    final havDist13 = MathUtils.havDistance(lat1, lat3, lng1 - lng3);
    if (havDist13 <= havTolerance) {
      return true;
    }

    final havDist23 = MathUtils.havDistance(lat2, lat3, lng2 - lng3);
    if (havDist23 <= havTolerance) {
      return true;
    }

    final sinBearing = _sinDeltaBearing(lat1, lng1, lat2, lng2, lat3, lng3);
    final sinDist13 = MathUtils.sinFromHav(havDist13);
    final havCrossTrack = MathUtils.havFromSin(sinDist13 * sinBearing);
    if (havCrossTrack > havTolerance) {
      return false;
    }

    final havDist12 = MathUtils.havDistance(lat1, lat2, lng1 - lng2);
    final term = havDist12 + havCrossTrack * (1 - 2 * havDist12);
    if (havDist13 > term || havDist23 > term) {
      return false;
    }

    if (havDist12 < 0.74) {
      return true;
    }

    final cosCrossTrack = 1 - 2 * havCrossTrack;
    final havAlongTrack13 = (havDist13 - havCrossTrack) / cosCrossTrack;
    final havAlongTrack23 = (havDist23 - havCrossTrack) / cosCrossTrack;
    final sinSumAlongTrack =
        MathUtils.sinSumFromHav(havAlongTrack13, havAlongTrack23);

    // Compare with half-circle == pi using sign of sin().
    return sinSumAlongTrack > 0;
  }

  /// Simplifies the given [poly] (polyline or polygon) using the Douglas-Peucker
  /// decimation algorithm. Increasing the [tolerance] will result in fewer points
  /// in the simplified polyline or polygon.
  ///
  /// When providing a polygon as input, the first and last point of the list
  /// MUST have the same x and y (i.e., the polygon must be closed). If the
  /// input polygon is not closed, the resulting polygon may not be fully
  /// simplified.
  ///
  /// The time complexity of Douglas-Peucker is O(n²), so take care that you do
  /// not call this algorithm too frequently in your code.
  ///
  /// Returns a simplified poly produced by the Douglas-Peucker algorithm.
  ///
  /// Note: this method does **not** mutate the input list.
  static List<Point<double>> simplify(
      List<Point<double>> poly, double tolerance) {
    final n = poly.length;
    if (n < 1) {
      throw ArgumentError('Polyline must have at least 1 point');
    }
    if (tolerance <= 0) {
      throw ArgumentError('Tolerance must be greater than zero');
    }

    final closedPolygon = isClosedPolygon(poly);

    // Work on a copy to avoid mutating the caller's list.
    final working = List<Point<double>>.of(poly);

    if (closedPolygon) {
      // Add a small offset to the last point for Douglas-Peucker on polygons
      // (see android-maps-utils #201).
      final lastPoint = working[working.length - 1];
      const offset = 0.00000000001;
      working[working.length - 1] =
          Point(lastPoint.x + offset, lastPoint.y + offset);
    }

    // Douglas-Peucker, iterative (stack-based) to avoid deep recursion.
    int idx = 0;
    int maxIdx = 0;
    final stack = Stack<List<int>>();
    final dists = List<double>.filled(n, 0.0);
    dists[0] = 1;
    dists[n - 1] = 1;
    double maxDist = 0.0;
    double dist = 0.0;
    List<int> current = [];

    if (n > 2) {
      stack.push([0, n - 1]);
      while (stack.isNotEmpty) {
        current = stack.pop();
        maxDist = 0;
        for (idx = current[0] + 1; idx < current[1]; ++idx) {
          dist = distanceToLine(
              working[idx], working[current[0]], working[current[1]]);
          if (dist > maxDist) {
            maxDist = dist;
            maxIdx = idx;
          }
        }
        if (maxDist > tolerance) {
          dists[maxIdx] = maxDist;
          stack.push([current[0], maxIdx]);
          stack.push([maxIdx, current[1]]);
        }
      }
    }

    // If the polygon was closed, restore the exact original last point.
    if (closedPolygon) {
      working[working.length - 1] = poly[poly.length - 1];
    }

    // Generate the simplified line.
    final simplifiedLine = <Point<double>>[];
    for (int i = 0; i < n; i++) {
      if (dists[i] != 0) {
        simplifiedLine.add(working[i]);
      }
    }
    return simplifiedLine;
  }

  /// Returns true if [poly] is a closed polygon (i.e., the first and last
  /// points are the same).
  static bool isClosedPolygon(List<Point<double>> poly) {
    if (poly.isEmpty) return false;
    final firstPoint = poly[0];
    final lastPoint = poly[poly.length - 1];
    return firstPoint == lastPoint;
  }

  /// Computes the distance on the sphere between the point [p] and the line
  /// segment from [start] to [end].
  ///
  /// Returns the distance in meters (assuming spherical earth).
  static double distanceToLine(final Point<double> p, final Point<double> start,
      final Point<double> end) {
    if (start == end) {
      return SphericalUtils.computeDistanceBetween(end, p);
    }

    final s0lat = SphericalUtils.toRadians(p.x);
    final s0lng = SphericalUtils.toRadians(p.y);
    final s1lat = SphericalUtils.toRadians(start.x);
    final s1lng = SphericalUtils.toRadians(start.y);
    final s2lat = SphericalUtils.toRadians(end.x);
    final s2lng = SphericalUtils.toRadians(end.y);

    final s2s1lat = s2lat - s1lat;
    final s2s1lng = s2lng - s1lng;
    final u = ((s0lat - s1lat) * s2s1lat + (s0lng - s1lng) * s2s1lng) /
        (s2s1lat * s2s1lat + s2s1lng * s2s1lng);

    if (u <= 0) {
      return SphericalUtils.computeDistanceBetween(p, start);
    } else if (u >= 1) {
      return SphericalUtils.computeDistanceBetween(p, end);
    }

    final latLng = Point(
      start.x + u * (end.x - start.x),
      start.y + u * (end.y - start.y),
    );
    return SphericalUtils.computeDistanceBetween(p, latLng);
  }

  /// Decodes an encoded path string into a sequence of points.
  ///
  /// Uses the [Encoded Polyline Algorithm](https://developers.google.com/maps/documentation/utilities/polylinealgorithm).
  static List<Point<double>> decode(final String encodedPath) {
    final len = encodedPath.length;

    final path = <Point<double>>[];
    int index = 0;
    int lat = 0;
    int lng = 0;

    while (index < len) {
      int result = 1;
      int shift = 0;
      int b;
      do {
        b = encodedPath.codeUnitAt(index++) - 63 - 1;
        result += b << shift;
        shift += 5;
      } while (b >= 0x1f);
      lat += (result & 1) != 0 ? ~(result >> 1) : (result >> 1);

      result = 1;
      shift = 0;
      do {
        b = encodedPath.codeUnitAt(index++) - 63 - 1;
        result += b << shift;
        shift += 5;
      } while (b >= 0x1f);
      lng += (result & 1) != 0 ? ~(result >> 1) : (result >> 1);

      path.add(Point(lat * 1e-5, lng * 1e-5));
    }

    return path;
  }

  /// Encodes a sequence of points into an encoded path string.
  ///
  /// Uses the [Encoded Polyline Algorithm](https://developers.google.com/maps/documentation/utilities/polylinealgorithm).
  static String encode(final List<Point<double>> path) {
    int lastLat = 0;
    int lastLng = 0;

    final result = StringBuffer();

    for (final point in path) {
      final lat = (point.x * 1e5).round();
      final lng = (point.y * 1e5).round();

      _encode(lat - lastLat, result);
      _encode(lng - lastLng, result);

      lastLat = lat;
      lastLng = lng;
    }
    return result.toString();
  }

  static void _encode(int v, StringBuffer result) {
    v = v < 0 ? ~(v << 1) : v << 1;
    while (v >= 0x20) {
      result.writeCharCode((0x20 | (v & 0x1f)) + 63);
      v >>= 5;
    }
    result.writeCharCode(v + 63);
  }
}
