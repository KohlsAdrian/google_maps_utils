import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';

/// A unified entry point for the google_maps_utils package.
///
/// Provides access to all utility classes through a single namespace,
/// giving a familiar Flutter-package feel:
///
/// ```dart
/// // Spherical geometry
/// GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
/// GoogleMapsUtils.spherical.computeHeading(from, to);
///
/// // Polyline utilities
/// GoogleMapsUtils.poly.decode(encodedPath);
/// GoogleMapsUtils.poly.containsLocationPoly(point, polygon);
///
/// // Math helpers
/// GoogleMapsUtils.math.clamp(value, 0, 10);
/// ```
class GoogleMapsUtils {
  GoogleMapsUtils._();

  /// Spherical geometry utilities: distances, headings, offsets,
  /// interpolation, areas, and bounding boxes.
  static const spherical = GoogleMapsUtilsSpherical._();

  /// Polyline and polygon utilities: encoding/decoding, simplification,
  /// point-in-polygon, and on-path/on-edge tests.
  static const poly = GoogleMapsUtilsPoly._();

  /// Low-level math helpers: haversine, Mercator projections, clamping,
  /// and wrapping.
  static const math = GoogleMapsUtilsMath._();
}

// =============================================================================
// SPHERICAL
// =============================================================================

/// Instance wrapper around [SphericalUtils] for use via [GoogleMapsUtils.spherical].
class GoogleMapsUtilsSpherical {
  const GoogleMapsUtilsSpherical._();

  /// Converts [deg] degrees to radians.
  double toRadians(double deg) => SphericalUtils.toRadians(deg);

  /// Converts [rad] radians to degrees.
  double toDegrees(double rad) => SphericalUtils.toDegrees(rad);

  /// Returns a 16-point compass cardinal abbreviation for the given [angle].
  String getCardinal(double angle) => SphericalUtils.getCardinal(angle);

  /// Creates a bounding box centered at ([x], [y]) with [radiusInMeters].
  GoogleMapsUtilsLatLngBounds toBounds(
          double x, double y, double radiusInMeters) =>
      SphericalUtils.toBounds(x, y, radiusInMeters);

  /// Computes the smallest bounding box that contains all [points].
  GoogleMapsUtilsLatLngBounds toBoundsFromPoints(List<Point<double>> points) =>
      SphericalUtils.toBoundsFromPoints(points);

  /// Returns the center point of the given [bounds].
  Point<double> centerFromLatLngBounds(GoogleMapsUtilsLatLngBounds bounds) =>
      SphericalUtils.centerFromLatLngBounds(bounds);

  /// Returns the heading from [from] to [to] in degrees \[-180, 180).
  double computeHeading(Point<double> from, Point<double> to) =>
      SphericalUtils.computeHeading(from, to);

  /// Returns the point at [distance] meters from [from] along [heading].
  Point<double> computeOffset(
          Point<double> from, double distance, double heading) =>
      SphericalUtils.computeOffset(from, distance, heading);

  /// Returns the origin given a destination [to], [distance], and [heading].
  Point<double>? computeOffsetOrigin(
          Point<double> to, double distance, double heading) =>
      SphericalUtils.computeOffsetOrigin(to, distance, heading);

  /// Returns the point at [fraction] of the way between [from] and [to].
  Point<double> interpolate(
          Point<double> from, Point<double> to, double fraction) =>
      SphericalUtils.interpolate(from, to, fraction);

  /// Returns distance on the unit sphere (radians in, radians out).
  double distanceRadians(double lat1, double lng1, double lat2, double lng2) =>
      SphericalUtils.distanceRadians(lat1, lng1, lat2, lng2);

  /// Returns the angle between [from] and [to], in radians.
  double computeAngleBetween(Point<double> from, Point<double> to) =>
      SphericalUtils.computeAngleBetween(from, to);

  /// Returns the distance between [from] and [to], in meters.
  double computeDistanceBetween(Point<double> from, Point<double> to) =>
      SphericalUtils.computeDistanceBetween(from, to);

  /// Returns the length of the given [path], in meters.
  double computeLength(List<Point<double>> path) =>
      SphericalUtils.computeLength(path);

  /// Returns the area enclosed by [path], in square meters.
  double computeArea(List<Point<double>> path) =>
      SphericalUtils.computeArea(path);

  /// Returns the signed area of [path] (positive = counter-clockwise).
  double computeSignedArea(List<Point<double>> path) =>
      SphericalUtils.computeSignedArea(path);

  /// Splits [bounds] into a grid of [division]² sub-bounds.
  List<GoogleMapsUtilsLatLngBounds> toSubBounds(
          GoogleMapsUtilsLatLngBounds bounds,
          {int division = 2}) =>
      SphericalUtils.toSubBounds(bounds, division: division);
}

// =============================================================================
// POLY
// =============================================================================

/// Instance wrapper around [PolyUtils] for use via [GoogleMapsUtils.poly].
class GoogleMapsUtilsPoly {
  const GoogleMapsUtilsPoly._();

  /// Checks whether [point] is inside [polygon] on the sphere.
  bool containsLocationPoly(Point<double> point, List<Point<double>> polygon,
          {bool geodesic = false}) =>
      PolyUtils.containsLocationPoly(point, polygon, geodesic: geodesic);

  /// Whether [point] lies on [polygon]'s edge within [tolerance] meters.
  bool isLocationOnEdgeTolerance(Point<double> point,
          List<Point<double>> polygon, bool geodesic, double tolerance) =>
      PolyUtils.isLocationOnEdgeTolerance(point, polygon, geodesic, tolerance);

  /// Whether [point] lies on [polygon]'s edge (0.1m tolerance).
  bool isLocationOnEdge(
          Point<double> point, List<Point<double>> polygon, bool geodesic) =>
      PolyUtils.isLocationOnEdge(point, polygon, geodesic);

  /// Whether [point] lies on [polyline] within [tolerance] meters.
  bool isLocationOnPathTolerance(Point<double> point,
          List<Point<double>> polyline, bool geodesic, double tolerance) =>
      PolyUtils.isLocationOnPathTolerance(point, polyline, geodesic, tolerance);

  /// Whether [point] lies on [polyline] (0.1m tolerance).
  bool isLocationOnPath(
          Point<double> point, List<Point<double>> polyline, bool geodesic) =>
      PolyUtils.isLocationOnPath(point, polyline, geodesic);

  /// Segment index of [point] on [poly] with custom [tolerance], or -1.
  int locationIndexOnPathTolerance(Point<double> point,
          List<Point<double>> poly, bool geodesic, double tolerance) =>
      PolyUtils.locationIndexOnPathTolerance(point, poly, geodesic, tolerance);

  /// Segment index of [point] on [polyline], or -1.
  int locationIndexOnPath(
          Point<double> point, List<Point<double>> polyline, bool geodesic) =>
      PolyUtils.locationIndexOnPath(point, polyline, geodesic);

  /// Full control: segment index with [closed]/open and custom [toleranceEarth].
  int locationIndexOnEdgeOrPath(Point<double> point, List<Point<double>> poly,
          bool closed, bool geodesic, double toleranceEarth) =>
      PolyUtils.locationIndexOnEdgeOrPath(
          point, poly, closed, geodesic, toleranceEarth);

  /// Simplifies [poly] using Douglas-Peucker with [tolerance] in meters.
  List<Point<double>> simplify(List<Point<double>> poly, double tolerance) =>
      PolyUtils.simplify(poly, tolerance);

  /// Whether [poly] is a closed polygon (first == last).
  bool isClosedPolygon(List<Point<double>> poly) =>
      PolyUtils.isClosedPolygon(poly);

  /// Distance from [p] to the line segment from [start] to [end], in meters.
  double distanceToLine(
          Point<double> p, Point<double> start, Point<double> end) =>
      PolyUtils.distanceToLine(p, start, end);

  /// Decodes an encoded polyline string into points.
  List<Point<double>> decode(String encodedPath) =>
      PolyUtils.decode(encodedPath);

  /// Encodes points into an encoded polyline string.
  String encode(List<Point<double>> path) => PolyUtils.encode(path);
}

// =============================================================================
// MATH
// =============================================================================

/// Instance wrapper around [MathUtils] for use via [GoogleMapsUtils.math].
class GoogleMapsUtilsMath {
  const GoogleMapsUtilsMath._();

  /// Earth's mean radius in meters (IUGG).
  double get earthRadius => MathUtils.earthRadius;

  /// Restricts [x] to \[[low], [high]\].
  double clamp(double x, double low, double high) =>
      MathUtils.clamp(x, low, high);

  /// Wraps [n] into \[[min], [max]).
  double wrap(double n, double min, double max) => MathUtils.wrap(n, min, max);

  /// Non-negative remainder of [x] / [m].
  double mod(double x, double m) => MathUtils.mod(x, m);

  /// Mercator Y from latitude [lat] (in radians).
  double mercator(double lat) => MathUtils.mercator(lat);

  /// Latitude (in radians) from Mercator Y [y].
  double inverseMercator(double y) => MathUtils.inverseMercator(y);

  /// Haversine of [x] (angle in radians).
  double hav(double x) => MathUtils.hav(x);

  /// Inverse haversine.
  double arcHav(double x) => MathUtils.arcHav(x);

  /// sin(|x|) given hav(x).
  double sinFromHav(double h) => MathUtils.sinFromHav(h);

  /// hav(asin([x])).
  double havFromSin(double x) => MathUtils.havFromSin(x);

  /// sin(arcHav([x]) + arcHav([y])).
  double sinSumFromHav(double x, double y) => MathUtils.sinSumFromHav(x, y);

  /// Haversine distance on the unit sphere.
  double havDistance(double lat1, double lat2, double dLng) =>
      MathUtils.havDistance(lat1, lat2, dLng);
}
