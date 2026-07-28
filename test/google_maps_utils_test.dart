import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';
import 'package:test/test.dart';

void main() {
  group('Testing Google Maps Utils', () {
    final point = Point<double>(-31.623060136389135, -60.68669021129609);
    final from = Point<double>(0.0, 0.0);
    final to = Point<double>(10.0, 5.0);
    final randomPoint = Point<double>(-23.54545, -23.898098);
    final path = GoogleMapsUtils.poly.decode(
        'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD');

    /// Triangle
    final polygon = <Point<double>>[
      Point(-31.624115, -60.688734),
      Point(-31.624115, -60.684657),
      Point(-31.621594, -60.686717),
      Point(-31.624115, -60.688734),
    ];

    test('heading', () async {
      final heading = GoogleMapsUtils.spherical.computeHeading(from, to);
      expect(heading, 26.302486345342523);
    });

    test('angle', () async {
      final angle = GoogleMapsUtils.spherical.computeAngleBetween(from, to);
      expect(angle, 0.19493500057547358);
    });
    test('distance to line', () async {
      final distanceToAB =
          GoogleMapsUtils.poly.distanceToLine(randomPoint, from, to);
      expect(distanceToAB, 3675538.019968191);
    });
    test('simplified path', () async {
      final simplified = GoogleMapsUtils.poly.simplify(path, 5000);
      final encoded = GoogleMapsUtils.poly.encode(simplified);
      expect(encoded, 'wjiaFz`hgQcjIke\\t{d@|aOutJz|UokC}xWomJdjM');
    });
    test('distance between', () async {
      final distance =
          GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
      expect(distance, 1241932.5985192063);
    });

    test('computeLength returns total path distance', () async {
      final length = GoogleMapsUtils.spherical.computeLength(polygon);
      expect(length, closeTo(1066.7243813716, 1e-6));
    });
    test('point contains inside polygon', () async {
      final contains =
          GoogleMapsUtils.poly.containsLocationPoly(point, polygon);
      expect(contains, true);
    });
    test('bounds should split into 4 subBounds', () async {
      final bounds = GoogleMapsUtils.spherical.toBounds(
        randomPoint.x,
        randomPoint.y,
        1000,
      );
      final subBounds = GoogleMapsUtils.spherical.toSubBounds(bounds);
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
      expect(GoogleMapsUtils.poly.containsLocationPoly(Point(1.0, 2.0), lShape),
          isTrue);
      expect(GoogleMapsUtils.poly.containsLocationPoly(Point(1.5, 2.0), lShape),
          isTrue);
    });
    test('exterior point level with a vertex is outside', () {
      expect(
          GoogleMapsUtils.poly.containsLocationPoly(Point(-1.0, 2.0), lShape),
          isFalse);
      expect(GoogleMapsUtils.poly.containsLocationPoly(Point(3.0, 3.0), lShape),
          isFalse);
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
      expect(
          GoogleMapsUtils.poly.containsLocationPoly(Point(2.0, 1.0), pentagon),
          isTrue);
      expect(
          GoogleMapsUtils.poly.containsLocationPoly(Point(0.0, 1.0), pentagon),
          isFalse);
    });

    test('a vertex counts as inside', () {
      expect(GoogleMapsUtils.poly.containsLocationPoly(Point(4.0, 2.0), lShape),
          isTrue);
    });

    test('works whether the polygon is closed or open', () {
      final open = [Point(0.0, 0.0), Point(0.0, 4.0), Point(4.0, 0.0)];
      final closed = [...open, Point(0.0, 0.0)];
      final inside = Point(1.0, 1.0);
      expect(
          GoogleMapsUtils.poly.containsLocationPoly(inside, open), isTrue);
      expect(
          GoogleMapsUtils.poly.containsLocationPoly(inside, closed), isTrue);
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
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(0.0, 180.0), antimeridianBox),
          isTrue);
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(0.0, 175.0), antimeridianBox),
          isTrue);
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(0.0, -175.0), antimeridianBox),
          isTrue);
    });
    test('point on the far side of the world is outside', () {
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(0.0, 0.0), antimeridianBox),
          isFalse);
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(0.0, 160.0), antimeridianBox),
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
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(89.0, 0.0), polarCap),
          isTrue);
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(89.0, 0.0), polarCap, geodesic: true),
          isTrue);
    });
    test('point well south of the cap is outside', () {
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(50.0, 0.0), polarCap),
          isFalse);
    });

    test('geodesic flag is accepted and agrees on a small polygon', () {
      final tri = [Point(0.0, 0.0), Point(0.0, 4.0), Point(4.0, 0.0)];
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(1.0, 1.0), tri, geodesic: true),
          isTrue);
      expect(
          GoogleMapsUtils.poly
              .containsLocationPoly(Point(5.0, 5.0), tri, geodesic: true),
          isFalse);
    });
  });

  // Regression tests for the geodesic on-segment test.
  group('isLocationOnPath / isLocationOnEdge (geodesic regression)', () {
    final segment = [Point(0.0, 0.0), Point(0.0, 10.0)];

    test('point exactly on the segment is detected (geodesic)', () {
      expect(
          GoogleMapsUtils.poly.isLocationOnPath(Point(0.0, 5.0), segment, true),
          isTrue);
    });

    test('point exactly on the segment is detected (rhumb)', () {
      expect(GoogleMapsUtils.poly
          .isLocationOnPath(Point(0.0, 5.0), segment, false), isTrue);
    });

    test('point within tolerance of an endpoint is detected', () {
      expect(
          GoogleMapsUtils.poly.isLocationOnPath(Point(0.0, 0.0), segment, true),
          isTrue);
      expect(GoogleMapsUtils.poly
          .isLocationOnPath(Point(0.0, 10.0), segment, true), isTrue);
    });

    test('point clearly off the segment is rejected', () {
      expect(
          GoogleMapsUtils.poly.isLocationOnPath(Point(1.0, 5.0), segment, true),
          isFalse);
    });

    test('locationIndexOnPath returns the segment index', () {
      expect(
          GoogleMapsUtils.poly
              .locationIndexOnPath(Point(0.0, 5.0), segment, true),
          0);
      expect(
          GoogleMapsUtils.poly
              .locationIndexOnPath(Point(1.0, 5.0), segment, true),
          -1);
    });

    test('isLocationOnEdge includes the closing segment that path excludes',
        () {
      final square = [
        Point(0.0, 0.0),
        Point(0.0, 10.0),
        Point(10.0, 10.0),
        Point(10.0, 0.0),
      ];
      final onClosingEdge = Point(5.0, 0.0);
      expect(
          GoogleMapsUtils.poly.isLocationOnEdge(onClosingEdge, square, true),
          isTrue);
      expect(
          GoogleMapsUtils.poly.isLocationOnPath(onClosingEdge, square, true),
          isFalse);
    });
  });

  group('SphericalUtils', () {
    test('computeOffset moves the expected distance and heading', () {
      final origin = Point(0.0, 0.0);
      final moved = GoogleMapsUtils.spherical.computeOffset(origin, 1000, 90);
      expect(GoogleMapsUtils.spherical.computeDistanceBetween(origin, moved),
          closeTo(1000, 1e-6));
      expect(GoogleMapsUtils.spherical.computeHeading(origin, moved),
          closeTo(90, 1e-6));
    });

    test('computeOffsetOrigin inverts computeOffset', () {
      final origin = Point(10.0, 20.0);
      final dest = GoogleMapsUtils.spherical.computeOffset(origin, 5000, 33);
      final recovered =
          GoogleMapsUtils.spherical.computeOffsetOrigin(dest, 5000, 33)!;
      expect(recovered.x, closeTo(origin.x, 1e-6));
      expect(recovered.y, closeTo(origin.y, 1e-6));
    });

    test('interpolate at the equator returns the midpoint', () {
      final mid = GoogleMapsUtils.spherical
          .interpolate(Point(0.0, 0.0), Point(0.0, 10.0), 0.5);
      expect(mid.x, closeTo(0.0, 1e-9));
      expect(mid.y, closeTo(5.0, 1e-9));
    });

    test('computeLength equals computeDistanceBetween for a 2-point path', () {
      final a = Point(0.0, 0.0);
      final b = Point(0.0, 10.0);
      expect(GoogleMapsUtils.spherical.computeLength([a, b]),
          closeTo(GoogleMapsUtils.spherical.computeDistanceBetween(a, b), 1e-6));
      expect(GoogleMapsUtils.spherical.computeLength([a]), 0);
    });

    test('computeArea is positive and matches abs(signed area)', () {
      final triangle = [
        Point(0.0, 0.0),
        Point(0.0, 1.0),
        Point(1.0, 0.0),
        Point(0.0, 0.0),
      ];
      final reversed = triangle.reversed.toList();
      final area = GoogleMapsUtils.spherical.computeArea(triangle);
      expect(area, greaterThan(0));
      expect(
          area,
          closeTo(
              GoogleMapsUtils.spherical.computeSignedArea(triangle).abs(),
              1e-6));
      // Reversing the winding flips the sign but not the magnitude.
      expect(GoogleMapsUtils.spherical.computeSignedArea(reversed),
          closeTo(-GoogleMapsUtils.spherical.computeSignedArea(triangle), 1e-3));
    });

    test('getCardinal maps angles to compass points', () {
      expect(GoogleMapsUtils.spherical.getCardinal(0), 'N');
      expect(GoogleMapsUtils.spherical.getCardinal(45), 'NE');
      expect(GoogleMapsUtils.spherical.getCardinal(90), 'E');
      expect(GoogleMapsUtils.spherical.getCardinal(180), 'S');
      expect(GoogleMapsUtils.spherical.getCardinal(270), 'W');
      expect(GoogleMapsUtils.spherical.getCardinal(360), 'N');
      expect(GoogleMapsUtils.spherical.getCardinal(-90), 'W');
    });

    test('toBounds is centered on the origin point', () {
      final bounds = GoogleMapsUtils.spherical.toBounds(0.0, 0.0, 1000);
      final center = GoogleMapsUtils.spherical.centerFromLatLngBounds(bounds);
      expect(center.x, closeTo(0.0, 1e-6));
      expect(center.y, closeTo(0.0, 1e-6));
      expect(bounds.ne.x, greaterThan(bounds.sw.x));
      expect(bounds.ne.y, greaterThan(bounds.sw.y));
    });

    test('toBoundsFromPoints finds the NE/SW extremes', () {
      final pts = [
        Point(1.0, 2.0),
        Point(-3.0, 5.0),
        Point(4.0, -1.0),
      ];
      final bounds = GoogleMapsUtils.spherical.toBoundsFromPoints(pts);
      expect(bounds.ne.x, 4.0);
      expect(bounds.ne.y, 5.0);
      expect(bounds.sw.x, -3.0);
      expect(bounds.sw.y, -1.0);
    });
  });

  group('PolyUtils encode/decode', () {
    test('decode(encode(points)) round-trips', () {
      final points = [
        Point(38.5, -120.2),
        Point(40.7, -120.95),
        Point(43.252, -126.453),
      ];
      final decoded =
          GoogleMapsUtils.poly.decode(GoogleMapsUtils.poly.encode(points));
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
      expect(GoogleMapsUtils.poly.encode(points), '_p~iF~ps|U_ulLnnqC_mqNvxq`@');
    });

    test('isClosedPolygon', () {
      expect(
          GoogleMapsUtils.poly
              .isClosedPolygon([Point(0.0, 0.0), Point(1.0, 1.0), Point(0.0, 0.0)]),
          isTrue);
      expect(
          GoogleMapsUtils.poly
              .isClosedPolygon([Point(0.0, 0.0), Point(1.0, 1.0)]),
          isFalse);
      expect(GoogleMapsUtils.poly.isClosedPolygon([]), isFalse);
    });
  });

  group('MathUtils', () {
    test('clamp', () {
      expect(GoogleMapsUtils.math.clamp(5, 0, 10), 5);
      expect(GoogleMapsUtils.math.clamp(-1, 0, 10), 0);
      expect(GoogleMapsUtils.math.clamp(11, 0, 10), 10);
    });

    test('wrap', () {
      expect(GoogleMapsUtils.math.wrap(190, -180, 180), -170);
      expect(GoogleMapsUtils.math.wrap(-190, -180, 180), 170);
      expect(GoogleMapsUtils.math.wrap(45, -180, 180), 45);
    });

    test('mod is always non-negative', () {
      expect(GoogleMapsUtils.math.mod(5, 3), 2);
      expect(GoogleMapsUtils.math.mod(-1, 3), 2);
      expect(GoogleMapsUtils.math.mod(0, 3), 0);
    });

    test('hav and arcHav are inverses', () {
      expect(GoogleMapsUtils.math.hav(0), 0);
      expect(GoogleMapsUtils.math.arcHav(0), 0);
      for (final x in [0.1, 0.5, 1.0, 2.0]) {
        expect(
            GoogleMapsUtils.math.arcHav(GoogleMapsUtils.math.hav(x)),
            closeTo(x, 1e-12));
      }
    });

    test('mercator and inverseMercator are inverses', () {
      for (final lat in [-1.0, 0.0, 0.5, 1.0]) {
        expect(
            GoogleMapsUtils.math
                .inverseMercator(GoogleMapsUtils.math.mercator(lat)),
            closeTo(lat, 1e-12));
      }
    });
  });
}
