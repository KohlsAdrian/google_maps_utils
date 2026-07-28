// ============================================================================
// Google Maps Utils for Dart/Flutter — Example
// ============================================================================
//
// This example demonstrates all major features of the google_maps_utils package.
// Each section is self-contained so you can copy individual snippets into your
// own project.
//
// Coordinate convention used throughout:
//   Point.x = latitude  (north/south, range -90 to 90)
//   Point.y = longitude (east/west, range -180 to 180)
//
// Run this example with:
//   dart run example/example.dart
// ============================================================================

import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';

void main() {
  // --------------------------------------------------------------------------
  // 1. DISTANCE BETWEEN TWO POINTS
  // --------------------------------------------------------------------------
  // Computes the great-circle distance between two coordinates in meters.
  // Uses the Haversine formula internally for numerical stability.

  final paris = Point<double>(48.8566, 2.3522); // Paris, France
  final london = Point<double>(51.5074, -0.1278); // London, UK

  final distanceParisLondon =
      SphericalUtils.computeDistanceBetween(paris, london);
  print('=== Distance ===');
  print('Paris → London: ${distanceParisLondon.toStringAsFixed(2)} meters');
  // Expected: ~341,549 meters (~341 km)
  print('');

  // --------------------------------------------------------------------------
  // 2. HEADING (BEARING)
  // --------------------------------------------------------------------------
  // Returns the initial bearing from one point to another.
  // Result is in degrees clockwise from North, range [-180, 180).

  final heading = SphericalUtils.computeHeading(paris, london);
  print('=== Heading ===');
  print('Paris → London heading: ${heading.toStringAsFixed(4)}°');
  // A negative value means the heading is west of north.
  print('');

  // --------------------------------------------------------------------------
  // 3. COMPASS CARDINAL DIRECTION
  // --------------------------------------------------------------------------
  // Converts a heading angle to a 16-point compass abbreviation.
  // Useful for user-friendly direction display (e.g. "Head NW").

  final cardinal = SphericalUtils.getCardinal(heading);
  print('=== Cardinal ===');
  print('Heading ${heading.toStringAsFixed(1)}° = $cardinal');

  // All 16 compass points:
  // N, NNE, NE, ENE, E, ESE, SE, SSE, S, SSW, SW, WSW, W, WNW, NW, NNW
  print('0°   = ${SphericalUtils.getCardinal(0)}');
  print('90°  = ${SphericalUtils.getCardinal(90)}');
  print('180° = ${SphericalUtils.getCardinal(180)}');
  print('270° = ${SphericalUtils.getCardinal(270)}');
  print('');

  // --------------------------------------------------------------------------
  // 4. ANGLE BETWEEN POINTS
  // --------------------------------------------------------------------------
  // Returns the arc distance on the unit sphere, in radians.
  // Multiply by earth's radius to get meters (same as computeDistanceBetween).

  final angle = SphericalUtils.computeAngleBetween(paris, london);
  print('=== Angle ===');
  print('Paris → London angle: ${angle.toStringAsFixed(8)} radians');
  print(
      'Verify: angle × earthRadius = ${(angle * MathUtils.earthRadius).toStringAsFixed(2)} meters');
  print('');

  // --------------------------------------------------------------------------
  // 5. COMPUTE OFFSET (MOVE A POINT)
  // --------------------------------------------------------------------------
  // Given a starting point, distance (meters), and heading (degrees),
  // returns the destination point.
  // Useful for: "Where do I end up if I walk 1km north?"

  final startPoint = Point<double>(-23.5505, -46.6333); // São Paulo, Brazil

  // Move 5000 meters (5km) due north (heading = 0°)
  final movedNorth = SphericalUtils.computeOffset(startPoint, 5000.0, 0.0);
  print('=== Compute Offset ===');
  print('Start: (${startPoint.x}, ${startPoint.y})');
  print(
      'After 5km North: (${movedNorth.x.toStringAsFixed(6)}, ${movedNorth.y.toStringAsFixed(6)})');

  // Move 5000 meters east (heading = 90°)
  final movedEast = SphericalUtils.computeOffset(startPoint, 5000.0, 90.0);
  print(
      'After 5km East:  (${movedEast.x.toStringAsFixed(6)}, ${movedEast.y.toStringAsFixed(6)})');
  print('');

  // --------------------------------------------------------------------------
  // 6. COMPUTE OFFSET ORIGIN (REVERSE LOOKUP)
  // --------------------------------------------------------------------------
  // Given a destination, distance, and heading — find where you started.
  // Returns null if no valid solution exists.

  final origin = SphericalUtils.computeOffsetOrigin(movedNorth, 5000.0, 0.0);
  print('=== Compute Offset Origin ===');
  if (origin != null) {
    print(
        'Reverse of 5km North: (${origin.x.toStringAsFixed(6)}, ${origin.y.toStringAsFixed(6)})');
    print('Original start was:    (${startPoint.x}, ${startPoint.y})');
    // Should match closely!
  }
  print('');

  // --------------------------------------------------------------------------
  // 7. INTERPOLATION (SLERP)
  // --------------------------------------------------------------------------
  // Finds a point at a given fraction along the great-circle path between two
  // points. fraction=0 returns `from`, fraction=1 returns `to`.
  // Useful for animations or placing markers along a route.

  print('=== Interpolation ===');
  final midpoint = SphericalUtils.interpolate(paris, london, 0.5);
  print(
      'Midpoint Paris→London: (${midpoint.x.toStringAsFixed(4)}, ${midpoint.y.toStringAsFixed(4)})');

  // You can generate multiple waypoints along the path:
  print('10 waypoints along Paris → London:');
  for (int i = 0; i <= 10; i++) {
    final fraction = i / 10.0;
    final wp = SphericalUtils.interpolate(paris, london, fraction);
    print(
        '  ${(fraction * 100).toStringAsFixed(0)}%: (${wp.x.toStringAsFixed(4)}, ${wp.y.toStringAsFixed(4)})');
  }
  print('');

  // --------------------------------------------------------------------------
  // 8. PATH LENGTH
  // --------------------------------------------------------------------------
  // Computes the total distance along a path (list of points), in meters.
  // The path does not need to be closed.

  final route = <Point<double>>[
    Point(-23.5505, -46.6333), // São Paulo
    Point(-22.9068, -43.1729), // Rio de Janeiro
    Point(-19.9167, -43.9345), // Belo Horizonte
  ];

  final routeLength = SphericalUtils.computeLength(route);
  print('=== Path Length ===');
  print('São Paulo → Rio → BH: ${(routeLength / 1000).toStringAsFixed(1)} km');
  print('');

  // --------------------------------------------------------------------------
  // 9. POLYLINE DECODING
  // --------------------------------------------------------------------------
  // Google's Encoded Polyline Algorithm compresses coordinates into a string.
  // This is commonly returned by the Directions API and OSRM.
  // decode() converts the string back into a list of points.
  //
  // Reference: https://developers.google.com/maps/documentation/utilities/polylinealgorithm

  final encodedPolyline =
      'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD';

  final decodedPath = PolyUtils.decode(encodedPolyline);
  print('=== Polyline Decode ===');
  print('Encoded string length: ${encodedPolyline.length} chars');
  print('Decoded into ${decodedPath.length} points');
  print('First point: (${decodedPath.first.x}, ${decodedPath.first.y})');
  print('Last point:  (${decodedPath.last.x}, ${decodedPath.last.y})');
  print('');

  // --------------------------------------------------------------------------
  // 10. POLYLINE ENCODING
  // --------------------------------------------------------------------------
  // The reverse: encode a list of coordinates into a compact string.
  // Useful for sending paths to APIs or storing them efficiently.

  final reEncoded = PolyUtils.encode(decodedPath);
  print('=== Polyline Encode ===');
  print('Re-encoded: $reEncoded');
  print('Matches original: ${reEncoded == encodedPolyline}');
  print('');

  // --------------------------------------------------------------------------
  // 11. DOUGLAS-PEUCKER SIMPLIFICATION
  // --------------------------------------------------------------------------
  // Reduces the number of points in a polyline while preserving its shape.
  // The `tolerance` is in meters — a higher value removes more points.
  // Useful for: reducing payload size, improving rendering performance.

  final simplified = PolyUtils.simplify(decodedPath, 5000.0);
  final simplifiedEncoded = PolyUtils.encode(simplified);
  print('=== Simplification ===');
  print('Original points:   ${decodedPath.length}');
  print('Simplified points: ${simplified.length}');
  print('Simplified encoded: $simplifiedEncoded');
  print('');

  // --------------------------------------------------------------------------
  // 12. POINT-IN-POLYGON
  // --------------------------------------------------------------------------
  // Determines whether a point lies inside a polygon.
  // Works correctly for polygons of any size, including those that cross the
  // antimeridian (±180°) or enclose a pole.
  //
  // The polygon can be open (auto-closed) or explicitly closed (first == last).
  // A point on a vertex is considered inside.

  // Define a triangle polygon (closed — last point == first point)
  final triangle = <Point<double>>[
    Point(-31.624115, -60.688734),
    Point(-31.624115, -60.684657),
    Point(-31.621594, -60.686717),
    Point(-31.624115, -60.688734), // closing point (optional)
  ];

  final insidePoint = Point<double>(-31.623060, -60.686690);
  final outsidePoint = Point<double>(-31.620000, -60.680000);

  print('=== Point-in-Polygon ===');
  print(
      'Inside point:  ${PolyUtils.containsLocationPoly(insidePoint, triangle)}'); // true
  print(
      'Outside point: ${PolyUtils.containsLocationPoly(outsidePoint, triangle)}'); // false

  // With geodesic edges (great-circle segments instead of rhumb lines):
  print(
      'Inside (geodesic): ${PolyUtils.containsLocationPoly(insidePoint, triangle, geodesic: true)}');
  print('');

  // --------------------------------------------------------------------------
  // 13. POINT-ON-PATH DETECTION
  // --------------------------------------------------------------------------
  // Checks if a point lies on or near a polyline within a tolerance.
  // Default tolerance is 0.1 meters.
  // The `geodesic` flag determines whether edges are great-circle or rhumb.

  final equatorPath = <Point<double>>[
    Point(0.0, 0.0),
    Point(0.0, 10.0),
  ];

  final onPath = Point<double>(0.0, 5.0); // exactly on the equator segment
  final offPath = Point<double>(1.0, 5.0); // 1° north — ~111km away

  print('=== Point-on-Path ===');
  print(
      'Point on equator segment: ${PolyUtils.isLocationOnPath(onPath, equatorPath, true)}'); // true
  print(
      'Point off the segment:    ${PolyUtils.isLocationOnPath(offPath, equatorPath, true)}'); // false

  // Get which segment index the point falls on:
  final segIdx = PolyUtils.locationIndexOnPath(onPath, equatorPath, true);
  print('Segment index: $segIdx'); // 0 (between equatorPath[0] and [1])
  print('');

  // --------------------------------------------------------------------------
  // 14. POINT-ON-EDGE (POLYGON BOUNDARY)
  // --------------------------------------------------------------------------
  // Like isLocationOnPath, but includes the closing segment between the last
  // and first points (i.e., treats the list as a closed polygon boundary).

  final square = <Point<double>>[
    Point(0.0, 0.0),
    Point(0.0, 10.0),
    Point(10.0, 10.0),
    Point(10.0, 0.0),
  ];

  // This point sits on the closing edge (from (10,0) back to (0,0))
  final edgePoint = Point<double>(5.0, 0.0);

  print('=== Point-on-Edge ===');
  print(
      'On polygon edge: ${PolyUtils.isLocationOnEdge(edgePoint, square, true)}'); // true
  print(
      'On open path:    ${PolyUtils.isLocationOnPath(edgePoint, square, true)}'); // false (closing edge not included)
  print('');

  // --------------------------------------------------------------------------
  // 15. DISTANCE TO A LINE SEGMENT
  // --------------------------------------------------------------------------
  // Computes the shortest spherical distance from a point to a line segment.
  // Useful for finding the closest point on a route to a given location.

  final randomPoint = Point<double>(-23.54545, -23.898098);
  final segStart = Point<double>(0.0, 0.0);
  final segEnd = Point<double>(10.0, 5.0);

  final distToLine = PolyUtils.distanceToLine(randomPoint, segStart, segEnd);
  print('=== Distance to Line ===');
  print('Distance: ${distToLine.toStringAsFixed(2)} meters');
  print('');

  // --------------------------------------------------------------------------
  // 16. COMPUTE AREA
  // --------------------------------------------------------------------------
  // Returns the area enclosed by a polygon in square meters.
  // computeSignedArea returns a positive value for counter-clockwise winding,
  // negative for clockwise. computeArea returns the absolute value.

  final area = SphericalUtils.computeArea(triangle);
  final signedArea = SphericalUtils.computeSignedArea(triangle);
  print('=== Area ===');
  print('Area: ${area.toStringAsFixed(2)} m²');
  print('Signed area: ${signedArea.toStringAsFixed(2)} m²');
  print('Winding: ${signedArea > 0 ? "counter-clockwise" : "clockwise"}');
  print('');

  // --------------------------------------------------------------------------
  // 17. BOUNDING BOXES
  // --------------------------------------------------------------------------
  // Create axis-aligned bounding boxes from points or center+radius.
  // Useful for map camera fitting, spatial queries, and tile subdivision.

  print('=== Bounding Boxes ===');

  // From a center point and radius (meters):
  final bounds = SphericalUtils.toBounds(-23.55, -46.63, 1000.0);
  print('Bounds from center+radius:');
  print(
      '  NE: (${bounds.northEast.x.toStringAsFixed(6)}, ${bounds.northEast.y.toStringAsFixed(6)})');
  print(
      '  SW: (${bounds.southWest.x.toStringAsFixed(6)}, ${bounds.southWest.y.toStringAsFixed(6)})');

  // From a list of points (finds the min/max lat/lng):
  final boundsFromPts = SphericalUtils.toBoundsFromPoints(route);
  print('Bounds from points:');
  print(
      '  NE: (${boundsFromPts.northEast.x.toStringAsFixed(4)}, ${boundsFromPts.northEast.y.toStringAsFixed(4)})');
  print(
      '  SW: (${boundsFromPts.southWest.x.toStringAsFixed(4)}, ${boundsFromPts.southWest.y.toStringAsFixed(4)})');

  // Center of a bounding box:
  final center = SphericalUtils.centerFromLatLngBounds(boundsFromPts);
  print(
      'Center: (${center.x.toStringAsFixed(4)}, ${center.y.toStringAsFixed(4)})');
  print('');

  // --------------------------------------------------------------------------
  // 18. SUB-BOUNDS (GRID TILING)
  // --------------------------------------------------------------------------
  // Splits a bounding box into a grid of smaller bounding boxes.
  // division=2 → 4 tiles, division=3 → 9 tiles, etc.
  // Useful for: paginated map loading, spatial partitioning.

  final tiles = SphericalUtils.toSubBounds(boundsFromPts, division: 3);
  print('=== Sub-Bounds ===');
  print('Split into ${tiles.length} tiles (3×3 grid):');
  for (int i = 0; i < tiles.length; i++) {
    final t = tiles[i];
    print(
        '  Tile $i: NE(${t.northEast.x.toStringAsFixed(4)}, ${t.northEast.y.toStringAsFixed(4)}) '
        'SW(${t.southWest.x.toStringAsFixed(4)}, ${t.southWest.y.toStringAsFixed(4)})');
  }
  print('');

  // --------------------------------------------------------------------------
  // 19. CLOSED POLYGON CHECK
  // --------------------------------------------------------------------------
  // A simple utility to check whether a polyline is closed (first == last).

  print('=== Closed Polygon Check ===');
  print('Triangle is closed: ${PolyUtils.isClosedPolygon(triangle)}'); // true
  print('Route is closed:    ${PolyUtils.isClosedPolygon(route)}'); // false
  print('');

  // --------------------------------------------------------------------------
  // 20. TOLERANCE VARIANTS
  // --------------------------------------------------------------------------
  // isLocationOnPath and isLocationOnEdge have default 0.1m tolerance.
  // You can specify a custom tolerance (in meters) for less/more strictness.

  print('=== Tolerance Variants ===');

  // Point that's ~11 meters off the equator (0.0001° lat ≈ 11m)
  final nearPoint = Point<double>(0.0001, 5.0);

  // Default tolerance (0.1m) — too far away:
  print(
      'Near point (0.1m tolerance):  ${PolyUtils.isLocationOnPathTolerance(nearPoint, equatorPath, true, 0.1)}'); // false

  // Custom tolerance of 20 meters — close enough:
  print(
      'Near point (20m tolerance):   ${PolyUtils.isLocationOnPathTolerance(nearPoint, equatorPath, true, 20.0)}'); // true

  // Edge with custom tolerance:
  print(
      'Edge (50m tolerance):         ${PolyUtils.isLocationOnEdgeTolerance(nearPoint, square, true, 20.0)}'); // true

  // locationIndexOnPath with tolerance:
  final idxTol = PolyUtils.locationIndexOnPathTolerance(
      nearPoint, equatorPath, true, 20.0);
  print('Segment index (20m tol): $idxTol'); // 0
  print('');

  // --------------------------------------------------------------------------
  // 21. LOCATION INDEX ON EDGE OR PATH (ADVANCED)
  // --------------------------------------------------------------------------
  // The underlying method that powers both isLocationOnPath and isLocationOnEdge.
  // The `closed` parameter controls whether the closing segment is included.

  print('=== locationIndexOnEdgeOrPath ===');

  // closed=false → acts like isLocationOnPath (no closing segment)
  final idxOpen =
      PolyUtils.locationIndexOnEdgeOrPath(edgePoint, square, false, true, 0.1);
  print('Edge point, open path:   index=$idxOpen'); // -1 (not found)

  // closed=true → acts like isLocationOnEdge (closing segment included)
  final idxClosed =
      PolyUtils.locationIndexOnEdgeOrPath(edgePoint, square, true, true, 0.1);
  print('Edge point, closed path: index=$idxClosed'); // 3 (on closing segment)
  print('');

  // --------------------------------------------------------------------------
  // 22. MATH UTILS (LOW-LEVEL HELPERS)
  // --------------------------------------------------------------------------
  // MathUtils provides the building blocks used internally by SphericalUtils
  // and PolyUtils. You can use them directly for custom calculations.

  print('=== MathUtils ===');

  // Earth's mean radius (IUGG standard, meters):
  print('Earth radius: ${MathUtils.earthRadius} meters');

  // Clamp: restricts a value to [low, high]
  print('clamp(15, 0, 10): ${MathUtils.clamp(15, 0, 10)}'); // 10
  print('clamp(-5, 0, 10): ${MathUtils.clamp(-5, 0, 10)}'); // 0
  print('clamp(5, 0, 10):  ${MathUtils.clamp(5, 0, 10)}'); // 5

  // Wrap: wraps a value into the range [min, max)
  // Commonly used for longitude wrapping [-180, 180)
  print('wrap(190, -180, 180):  ${MathUtils.wrap(190, -180, 180)}'); // -170
  print('wrap(-200, -180, 180): ${MathUtils.wrap(-200, -180, 180)}'); // 160

  // Mod: non-negative modulo (unlike Dart's %, which can return negatives)
  print('mod(-1, 360):  ${MathUtils.mod(-1, 360)}'); // 359
  print('mod(361, 360): ${MathUtils.mod(361, 360)}'); // 1

  // Mercator projection (latitude in radians → mercator Y, and inverse)
  final latRad = SphericalUtils.toRadians(45.0); // 45° latitude
  final mercY = MathUtils.mercator(latRad);
  final latBack = MathUtils.inverseMercator(mercY);
  print('mercator(45° in rad):    ${mercY.toStringAsFixed(8)}');
  print(
      'inverseMercator(y):      ${SphericalUtils.toDegrees(latBack).toStringAsFixed(4)}°'); // 45°

  // Haversine and inverse haversine
  // hav(θ) = (1 - cos(θ)) / 2 = sin²(θ/2)
  final h = MathUtils.hav(1.0); // hav(1 radian)
  final recovered = MathUtils.arcHav(h); // should give back 1.0
  print('hav(1.0):    ${h.toStringAsFixed(8)}');
  print('arcHav(h):   ${recovered.toStringAsFixed(8)}'); // 1.0

  // havDistance: haversine of the central angle between two points (on unit sphere)
  // Arguments are in radians: lat1, lat2, dLng (difference in longitude)
  final lat1 = SphericalUtils.toRadians(48.8566); // Paris lat
  final lat2 = SphericalUtils.toRadians(51.5074); // London lat
  final dLng =
      SphericalUtils.toRadians(2.3522) - SphericalUtils.toRadians(-0.1278);
  final havDist = MathUtils.havDistance(lat1, lat2, dLng);
  print('havDistance(Paris, London): ${havDist.toStringAsFixed(10)}');
  // To get actual distance: arcHav(havDist) * earthRadius
  final distFromHav = MathUtils.arcHav(havDist) * MathUtils.earthRadius;
  print('Distance from hav: ${distFromHav.toStringAsFixed(2)} meters');
  print('');

  // --------------------------------------------------------------------------
  // 23. DEGREE/RADIAN CONVERSIONS
  // --------------------------------------------------------------------------
  // SphericalUtils exposes toRadians and toDegrees for convenience.

  print('=== Degree/Radian Conversions ===');
  print('90° → radians: ${SphericalUtils.toRadians(90.0)}'); // π/2
  print('π radians → degrees: ${SphericalUtils.toDegrees(pi)}'); // 180
  print('');

  // --------------------------------------------------------------------------
  // 24. DISTANCE IN RADIANS (UNIT SPHERE)
  // --------------------------------------------------------------------------
  // distanceRadians returns the arc length on the unit sphere between two
  // points given in radians. Multiply by earthRadius for meters.
  // This is what computeAngleBetween uses internally.

  print('=== Distance Radians ===');
  final dRad = SphericalUtils.distanceRadians(
    SphericalUtils.toRadians(paris.x), // lat1
    SphericalUtils.toRadians(paris.y), // lng1
    SphericalUtils.toRadians(london.x), // lat2
    SphericalUtils.toRadians(london.y), // lng2
  );
  print('distanceRadians(Paris, London): ${dRad.toStringAsFixed(8)} rad');
  print(
      'Same as computeAngleBetween:    ${SphericalUtils.computeAngleBetween(paris, london).toStringAsFixed(8)} rad');
  print('');

  // --------------------------------------------------------------------------
  // 25. SIGNED AREA WITH CUSTOM RADIUS
  // --------------------------------------------------------------------------
  // computeSignedAreaTest lets you compute signed area on a sphere of any
  // radius (not just Earth). Useful for unit testing or non-Earth spheres.

  print('=== Signed Area (custom radius) ===');
  // Area on a unit sphere (radius = 1):
  final unitArea = SphericalUtils.computeSignedAreaTest(triangle, 1.0);
  print('Signed area on unit sphere: ${unitArea.toStringAsFixed(10)} sr');
  // Area on Earth:
  final earthArea =
      SphericalUtils.computeSignedAreaTest(triangle, MathUtils.earthRadius);
  print('Signed area on Earth: ${earthArea.toStringAsFixed(2)} m²');
  print(
      'Same as computeSignedArea: ${SphericalUtils.computeSignedArea(triangle).toStringAsFixed(2)} m²');
  print('');

  // --------------------------------------------------------------------------
  // DONE!
  // --------------------------------------------------------------------------
  print('✅ All examples completed successfully.');
}
