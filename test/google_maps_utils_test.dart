import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';
import 'package:test/test.dart';

void main() {
  group('Testing Google Maps Utils', () {
    final point = Point<double>(-31.623060136389135, -60.68669021129609);
    final from = Point<double>(0.0, 0.0);
    final to = Point<double>(10.0, 5.0);
    final randomPoint = Point<double>(-23.54545, -23.898098);
    final path = PolyUtils.decode(
        'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD');

    /// Triangle
    final polygon = <Point<double>>[
      Point(-31.624115, -60.688734),
      Point(-31.624115, -60.684657),
      Point(-31.621594, -60.686717),
      Point(-31.624115, -60.688734),
    ];

    test('heading', () async {
      final heading = SphericalUtils.computeHeading(from, to);
      expect(heading, 26.302486345342523);
    });

    test('angle', () async {
      final angle = SphericalUtils.computeAngleBetween(from, to);
      expect(angle, 0.19493500057547358);
    });
    test('distance to line', () async {
      final distanceToAB = PolyUtils.distanceToLine(randomPoint, from, to);
      expect(distanceToAB, 3675538.019968191);
    });
    test('simplified path', () async {
      final simplified = PolyUtils.simplify(path, 5000);
      final encoded = PolyUtils.encode(simplified);
      expect(encoded, 'wjiaFz`hgQcjIke\\t{d@|aOutJz|UokC}xWomJdjM');
    });
    test('distance between', () async {
      final distance = SphericalUtils.computeDistanceBetween(from, to);
      expect(distance, 1241932.5985192063);
    });

    test('distance in meters of points', () async {
      final distance = SphericalUtils.computeDistanceFromListOfPoints(polygon);
      expect(distance, 1066.724381371654);
    });
    test('point contains inside polygon', () async {
      final contains = PolyUtils.containsLocationPoly(point, polygon);
      expect(contains, true);
    });
    test('bounds should split into 4 subBounds', () async {
      final bounds = SphericalUtils.toBounds(
        randomPoint.x,
        randomPoint.y,
        1000,
      );
      final subBounds = SphericalUtils.toSubBounds(bounds);
      expect(subBounds.length, 4);
    });
  });

  // Regression tests for [issue #11](https://github.com/KohlsAdrian/google_maps_utils/issues/11): containsLocationPoly gave wrong results on
  // shapes with multiple vertices when the test point shared a latitude with a
  // vertex. The spherical port (android-maps-utils' containsLocation) handles
  // these correctly.
  group('containsLocationPoly (issue #11 — multi-vertex shapes)', () {
    // Concave "L" shape; y=2 is the latitude of the (2,2) and (4,2) vertices.
    final lShape = [
      Point(0.0, 0.0),
      Point(0.0, 4.0),
      Point(2.0, 4.0),
      Point(2.0, 2.0),
      Point(4.0, 2.0),
      Point(4.0, 0.0),
    ];
    test('interior point level with a vertex is inside', () {
      expect(PolyUtils.containsLocationPoly(Point(1.0, 2.0), lShape), isTrue);
      expect(PolyUtils.containsLocationPoly(Point(1.5, 2.0), lShape), isTrue);
    });
    test('exterior point level with a vertex is outside', () {
      expect(PolyUtils.containsLocationPoly(Point(-1.0, 2.0), lShape), isFalse);
      expect(PolyUtils.containsLocationPoly(Point(3.0, 3.0), lShape), isFalse);
    });

    // Pentagon; y=1 is the latitude of the (4,1) vertex.
    final pentagon = [
      Point(0.0, 2.0),
      Point(2.0, 0.0),
      Point(4.0, 1.0),
      Point(3.0, 4.0),
      Point(1.0, 4.0),
    ];
    test('pentagon: point level with the (4,1) vertex', () {
      expect(PolyUtils.containsLocationPoly(Point(2.0, 1.0), pentagon), isTrue);
      expect(
          PolyUtils.containsLocationPoly(Point(0.0, 1.0), pentagon), isFalse);
    });

    test('a vertex counts as inside', () {
      expect(PolyUtils.containsLocationPoly(Point(4.0, 2.0), lShape), isTrue);
    });

    test('works whether the polygon is closed or open', () {
      final open = [Point(0.0, 0.0), Point(0.0, 4.0), Point(4.0, 0.0)];
      final closed = [...open, Point(0.0, 0.0)];
      final inside = Point(1.0, 1.0);
      expect(PolyUtils.containsLocationPoly(inside, open), isTrue);
      expect(PolyUtils.containsLocationPoly(inside, closed), isTrue);
    });
  });

  // The spherical algorithm is correct for polygons that the old planar test
  // could not handle: those crossing the antimeridian or enclosing a pole.
  group('containsLocationPoly (spherical: antimeridian & poles)', () {
    // 20°-wide box straddling the 180° meridian (lng 170 → 180 → -170).
    final antimeridianBox = [
      Point(-10.0, 170.0),
      Point(10.0, 170.0),
      Point(10.0, -170.0),
      Point(-10.0, -170.0),
    ];
    test('point on the antimeridian is inside the box across the seam', () {
      expect(PolyUtils.containsLocationPoly(Point(0.0, 180.0), antimeridianBox),
          isTrue);
      expect(PolyUtils.containsLocationPoly(Point(0.0, 175.0), antimeridianBox),
          isTrue);
      expect(
          PolyUtils.containsLocationPoly(Point(0.0, -175.0), antimeridianBox),
          isTrue);
    });
    test('point on the far side of the world is outside', () {
      expect(PolyUtils.containsLocationPoly(Point(0.0, 0.0), antimeridianBox),
          isFalse);
      expect(PolyUtils.containsLocationPoly(Point(0.0, 160.0), antimeridianBox),
          isFalse);
    });

    // Cap around the North Pole: a ring of vertices at latitude 70°.
    final polarCap = [
      Point(70.0, 0.0),
      Point(70.0, 90.0),
      Point(70.0, 180.0),
      Point(70.0, -90.0),
    ];
    test('point near the pole is inside the polar cap', () {
      expect(
          PolyUtils.containsLocationPoly(Point(89.0, 0.0), polarCap), isTrue);
      expect(
          PolyUtils.containsLocationPoly(Point(89.0, 0.0), polarCap,
              geodesic: true),
          isTrue);
    });
    test('point well south of the cap is outside', () {
      expect(
          PolyUtils.containsLocationPoly(Point(50.0, 0.0), polarCap), isFalse);
    });

    test('geodesic flag is accepted and agrees on a small polygon', () {
      final tri = [Point(0.0, 0.0), Point(0.0, 4.0), Point(4.0, 0.0)];
      expect(
          PolyUtils.containsLocationPoly(Point(1.0, 1.0), tri, geodesic: true),
          isTrue);
      expect(
          PolyUtils.containsLocationPoly(Point(5.0, 5.0), tri, geodesic: true),
          isFalse);
    });
  });

  // Regression tests for the geodesic on-segment test. Before the fix to
  // PolyUtils._isOnSegmentGC, these all returned the wrong answer (the three
  // early-return branches were inverted), so isLocationOnPath / isLocationOnEdge
  // reported that a point sitting exactly on the line was NOT on it.
  group('isLocationOnPath / isLocationOnEdge (geodesic regression)', () {
    // A straight equatorial segment: the great circle between two equator
    // points is the equator itself, so points at lat 0 between them are
    // exactly on the path.
    final segment = [Point(0.0, 0.0), Point(0.0, 10.0)];

    test('point exactly on the segment is detected (geodesic)', () {
      expect(
          PolyUtils.isLocationOnPath(Point(0.0, 5.0), segment, true), isTrue);
    });

    test('point exactly on the segment is detected (rhumb)', () {
      expect(
          PolyUtils.isLocationOnPath(Point(0.0, 5.0), segment, false), isTrue);
    });

    test('point within tolerance of an endpoint is detected', () {
      expect(
          PolyUtils.isLocationOnPath(Point(0.0, 0.0), segment, true), isTrue);
      expect(
          PolyUtils.isLocationOnPath(Point(0.0, 10.0), segment, true), isTrue);
    });

    test('point clearly off the segment is rejected', () {
      // ~111 km north of the path, far beyond the default 0.1 m tolerance.
      expect(
          PolyUtils.isLocationOnPath(Point(1.0, 5.0), segment, true), isFalse);
    });

    test('locationIndexOnPath returns the segment index', () {
      expect(PolyUtils.locationIndexOnPath(Point(0.0, 5.0), segment, true), 0);
      expect(PolyUtils.locationIndexOnPath(Point(1.0, 5.0), segment, true), -1);
    });

    test('isLocationOnEdge includes the closing segment that path excludes',
        () {
      // Open square; the closing edge runs along the prime meridian from
      // (10,0) back to (0,0). The point (5,0) lies on that closing edge only.
      final square = [
        Point(0.0, 0.0),
        Point(0.0, 10.0),
        Point(10.0, 10.0),
        Point(10.0, 0.0),
      ];
      final onClosingEdge = Point(5.0, 0.0);
      expect(PolyUtils.isLocationOnEdge(onClosingEdge, square, true), isTrue);
      expect(PolyUtils.isLocationOnPath(onClosingEdge, square, true), isFalse);
    });
  });

  group('SphericalUtils', () {
    test('computeOffset moves the expected distance and heading', () {
      final origin = Point(0.0, 0.0);
      final moved = SphericalUtils.computeOffset(origin, 1000, 90);
      expect(SphericalUtils.computeDistanceBetween(origin, moved),
          closeTo(1000, 1e-6));
      expect(SphericalUtils.computeHeading(origin, moved), closeTo(90, 1e-6));
    });

    test('computeOffsetOrigin inverts computeOffset', () {
      final origin = Point(10.0, 20.0);
      final dest = SphericalUtils.computeOffset(origin, 5000, 33);
      final recovered = SphericalUtils.computeOffsetOrigin(dest, 5000, 33)!;
      expect(recovered.x, closeTo(origin.x, 1e-6));
      expect(recovered.y, closeTo(origin.y, 1e-6));
    });

    test('interpolate at the equator returns the midpoint', () {
      final mid =
          SphericalUtils.interpolate(Point(0.0, 0.0), Point(0.0, 10.0), 0.5);
      expect(mid.x, closeTo(0.0, 1e-9));
      expect(mid.y, closeTo(5.0, 1e-9));
    });

    test('computeLength equals computeDistanceBetween for a 2-point path', () {
      final a = Point(0.0, 0.0);
      final b = Point(0.0, 10.0);
      expect(SphericalUtils.computeLength([a, b]),
          closeTo(SphericalUtils.computeDistanceBetween(a, b), 1e-6));
      expect(SphericalUtils.computeLength([a]), 0);
    });

    test('computeArea is positive and matches abs(signed area)', () {
      final triangle = [
        Point(0.0, 0.0),
        Point(0.0, 1.0),
        Point(1.0, 0.0),
        Point(0.0, 0.0),
      ];
      final reversed = triangle.reversed.toList();
      final area = SphericalUtils.computeArea(triangle);
      expect(area, greaterThan(0));
      expect(area,
          closeTo(SphericalUtils.computeSignedArea(triangle).abs(), 1e-6));
      // Reversing the winding flips the sign but not the magnitude.
      expect(SphericalUtils.computeSignedArea(reversed),
          closeTo(-SphericalUtils.computeSignedArea(triangle), 1e-3));
    });

    test('getCardinal maps angles to compass points', () {
      expect(SphericalUtils.getCardinal(0), 'N');
      expect(SphericalUtils.getCardinal(45), 'NE');
      expect(SphericalUtils.getCardinal(90), 'E');
      expect(SphericalUtils.getCardinal(180), 'S');
      expect(SphericalUtils.getCardinal(270), 'W');
      expect(SphericalUtils.getCardinal(360), 'N');
      expect(SphericalUtils.getCardinal(-90), 'W');
    });

    test('toBounds is centered on the origin point', () {
      final bounds = SphericalUtils.toBounds(0.0, 0.0, 1000);
      final center = SphericalUtils.centerFromLatLngBounds(bounds);
      expect(center.x, closeTo(0.0, 1e-6));
      expect(center.y, closeTo(0.0, 1e-6));
      expect(bounds.northEast.x, greaterThan(bounds.southWest.x));
      expect(bounds.northEast.y, greaterThan(bounds.southWest.y));
    });

    test('toBoundsFromPoints finds the NE/SW extremes', () {
      final pts = [
        Point(1.0, 2.0),
        Point(-3.0, 5.0),
        Point(4.0, -1.0),
      ];
      final bounds = SphericalUtils.toBoundsFromPoints(pts);
      expect(bounds.northEast.x, 4.0);
      expect(bounds.northEast.y, 5.0);
      expect(bounds.southWest.x, -3.0);
      expect(bounds.southWest.y, -1.0);
    });
  });

  group('PolyUtils encode/decode', () {
    test('decode(encode(points)) round-trips', () {
      final points = [
        Point(38.5, -120.2),
        Point(40.7, -120.95),
        Point(43.252, -126.453),
      ];
      final decoded = PolyUtils.decode(PolyUtils.encode(points));
      expect(decoded.length, points.length);
      for (int i = 0; i < points.length; i++) {
        expect(decoded[i].x, closeTo(points[i].x, 1e-5));
        expect(decoded[i].y, closeTo(points[i].y, 1e-5));
      }
    });

    test('encode produces the canonical Google polyline string', () {
      final points = [
        Point(38.5, -120.2),
        Point(40.7, -120.95),
        Point(43.252, -126.453),
      ];
      expect(PolyUtils.encode(points), '_p~iF~ps|U_ulLnnqC_mqNvxq`@');
    });

    test('isClosedPolygon', () {
      expect(
          PolyUtils.isClosedPolygon(
              [Point(0.0, 0.0), Point(1.0, 1.0), Point(0.0, 0.0)]),
          isTrue);
      expect(PolyUtils.isClosedPolygon([Point(0.0, 0.0), Point(1.0, 1.0)]),
          isFalse);
      expect(PolyUtils.isClosedPolygon([]), isFalse);
    });
  });

  group('MathUtils', () {
    test('clamp', () {
      expect(MathUtils.clamp(5, 0, 10), 5);
      expect(MathUtils.clamp(-1, 0, 10), 0);
      expect(MathUtils.clamp(11, 0, 10), 10);
    });

    test('wrap', () {
      expect(MathUtils.wrap(190, -180, 180), -170);
      expect(MathUtils.wrap(-190, -180, 180), 170);
      expect(MathUtils.wrap(45, -180, 180), 45);
    });

    test('mod is always non-negative', () {
      expect(MathUtils.mod(5, 3), 2);
      expect(MathUtils.mod(-1, 3), 2);
      expect(MathUtils.mod(0, 3), 0);
    });

    test('hav and arcHav are inverses', () {
      expect(MathUtils.hav(0), 0);
      expect(MathUtils.arcHav(0), 0);
      for (final x in [0.1, 0.5, 1.0, 2.0]) {
        expect(MathUtils.arcHav(MathUtils.hav(x)), closeTo(x, 1e-12));
      }
    });

    test('mercator and inverseMercator are inverses', () {
      for (final lat in [-1.0, 0.0, 0.5, 1.0]) {
        expect(MathUtils.inverseMercator(MathUtils.mercator(lat)),
            closeTo(lat, 1e-12));
      }
    });
  });
}
