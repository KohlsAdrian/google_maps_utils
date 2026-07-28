<p align="center">
  <img src="maps-icon.svg" width="120" alt="Google Maps Utils for Flutter">
</p>

<h1 align="center">Google Maps Utils for Dart/Flutter</h1>

<p align="center">
  A faithful Dart port of the core geometry classes from Google's
  <a href="https://github.com/googlemaps/android-maps-utils">android-maps-utils</a> library.
</p>

<p align="center">
  <a href="https://pub.dev/packages/google_maps_utils"><img src="https://img.shields.io/pub/v/google_maps_utils.svg?label=pub&color=blue" alt="Pub Version"></a>
  <a href="https://pub.dev/packages/google_maps_utils/score"><img src="https://img.shields.io/pub/points/google_maps_utils?color=2E8B57&label=pub%20points" alt="Pub Points"></a>
  <a href="https://pub.dev/packages/google_maps_utils"><img src="https://img.shields.io/pub/popularity/google_maps_utils?color=orange" alt="Popularity"></a>
  <a href="https://pub.dev/packages/google_maps_utils"><img src="https://img.shields.io/pub/likes/google_maps_utils?color=red&label=pub%20likes" alt="Pub Likes"></a>
  <a href="https://github.com/KohlsAdrian/google_maps_utils"><img src="https://img.shields.io/github/stars/KohlsAdrian/google_maps_utils?style=flat&color=gold" alt="GitHub Stars"></a>
  <a href="https://github.com/KohlsAdrian/google_maps_utils/network/members"><img src="https://img.shields.io/github/forks/KohlsAdrian/google_maps_utils?style=flat&color=slateblue" alt="GitHub Forks"></a>
  <a href="https://github.com/KohlsAdrian/google_maps_utils/issues"><img src="https://img.shields.io/github/issues/KohlsAdrian/google_maps_utils?color=coral" alt="GitHub Issues"></a>
  <a href="https://github.com/KohlsAdrian/google_maps_utils/blob/master/LICENSE"><img src="https://img.shields.io/github/license/KohlsAdrian/google_maps_utils?color=teal" alt="License"></a>
  <a href="https://github.com/KohlsAdrian/google_maps_utils/actions"><img src="https://github.com/KohlsAdrian/google_maps_utils/actions/workflows/dart.yml/badge.svg" alt="CI"></a>
</p>

---

## ✨ Features

| Namespace | What it does |
|-----------|--------------|
| **`GoogleMapsUtils.spherical`** | Distances, headings, offsets, interpolation, areas, and bounding boxes on a sphere |
| **`GoogleMapsUtils.poly`** | Polyline encoding/decoding, Douglas-Peucker simplification, point-in-polygon, and on-path/on-edge tests |
| **`GoogleMapsUtils.math`** | Low-level haversine, Mercator projections, clamping, and wrapping helpers |

All calculations use the same spherical algorithms as the original Android Java/Kotlin library — results are identical.

---

## 🚀 Getting Started

### Installation

Add to your `pubspec.yaml`:

```yaml
dependencies:
  google_maps_utils: ^5.0.0
```

Then run:

```bash
dart pub get
```

### Import

```dart
import 'package:google_maps_utils/google_maps_utils.dart';
```

### Quick Start

Use the `GoogleMapsUtils` class as the unified namespace for all operations:

```dart
// Spherical geometry
final distance = GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
final heading = GoogleMapsUtils.spherical.computeHeading(from, to);

// Polyline utilities
final path = GoogleMapsUtils.poly.decode(encodedString);
final simplified = GoogleMapsUtils.poly.simplify(path, 5000.0);

// Math helpers
final wrapped = GoogleMapsUtils.math.wrap(190.0, -180.0, 180.0); // -170.0

// Bounding boxes return a lightweight record: (ne: Point, sw: Point)
final bounds = GoogleMapsUtils.spherical.toBounds(-23.55, -46.63, 1000.0);
print(bounds.ne); // north-east corner
print(bounds.sw); // south-west corner
```

---

## 📖 Usage

### Distance Between Two Points

```dart
final from = Point<double>(0.0, 0.0);
final to = Point<double>(10.0, 5.0);

final distance = GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
// 1241932.5985192063 meters
```

### Heading (Bearing)

```dart
final heading = GoogleMapsUtils.spherical.computeHeading(from, to);
// 26.302486345342523 degrees (clockwise from North)
```

### Angle Between Points

```dart
final angle = GoogleMapsUtils.spherical.computeAngleBetween(from, to);
// 0.19493500057547358 radians (unit-sphere arc distance)
```

### Compute Offset (Move a Point)

```dart
// Move 1000 meters east (heading 90°) from the origin
final origin = Point<double>(0.0, 0.0);
final destination = GoogleMapsUtils.spherical.computeOffset(origin, 1000.0, 90.0);
// Returns the Point at the new location
```

### Compute Offset Origin (Reverse Lookup)

```dart
// Given a destination, distance, and heading — find where you started
final origin = GoogleMapsUtils.spherical.computeOffsetOrigin(destination, 1000.0, 90.0);
// Returns Point? (null if no solution exists)
```

### Interpolation (SLERP)

```dart
// Find the point 25% of the way between two locations
final quarter = GoogleMapsUtils.spherical.interpolate(from, to, 0.25);
```

### Path Length

```dart
final path = <Point<double>>[
  Point(0.0, 0.0),
  Point(1.0, 1.0),
  Point(2.0, 0.0),
];

final length = GoogleMapsUtils.spherical.computeLength(path);
// Total distance in meters along the path
```

### Compass Cardinal Direction

```dart
final cardinal = GoogleMapsUtils.spherical.getCardinal(45.0);
// 'NE'

final cardinal2 = GoogleMapsUtils.spherical.getCardinal(200.0);
// 'SSW'
```

### Polyline Decoding

```dart
// Decode a Google Encoded Polyline string into coordinates
final path = GoogleMapsUtils.poly.decode(
    'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA');
// Returns List<Point<double>> with 17 points
```

### Polyline Encoding

```dart
// Encode coordinates back to a compact string
final encoded = GoogleMapsUtils.poly.encode(path);
```

### Douglas-Peucker Simplification

```dart
// Reduce points while preserving shape (tolerance in meters)
final simplified = GoogleMapsUtils.poly.simplify(path, 5000.0);
// 17 points → 6 points
```

### Point-in-Polygon

```dart
final point = Point<double>(-31.623060, -60.686690);

final polygon = <Point<double>>[
  Point(-31.624115, -60.688734),
  Point(-31.624115, -60.684657),
  Point(-31.621594, -60.686717),
  Point(-31.624115, -60.688734),
];

final isInside = GoogleMapsUtils.poly.containsLocationPoly(point, polygon);
// true

// Also supports geodesic (great-circle) edges:
final isInsideGeodesic = GoogleMapsUtils.poly.containsLocationPoly(
  point, polygon, geodesic: true,
);
```

### Point-on-Path Detection

```dart
final route = <Point<double>>[
  Point(0.0, 0.0),
  Point(0.0, 10.0),
];

// Is the point on or near the polyline? (default tolerance: 0.1m)
final isOnPath = GoogleMapsUtils.poly.isLocationOnPath(
  Point(0.0, 5.0), route, true, // geodesic
);
// true

// Get which segment the point falls on
final segmentIndex = GoogleMapsUtils.poly.locationIndexOnPath(
  Point(0.0, 5.0), route, true,
);
// 0 (between route[0] and route[1])
```

### Point-on-Edge (Polygon Boundary)

```dart
final square = <Point<double>>[
  Point(0.0, 0.0),
  Point(0.0, 10.0),
  Point(10.0, 10.0),
  Point(10.0, 0.0),
];

// Includes the closing segment (last→first)
final isOnEdge = GoogleMapsUtils.poly.isLocationOnEdge(
  Point(5.0, 0.0), square, true,
);
// true
```

### Distance to a Line Segment

```dart
final p = Point<double>(-23.54545, -23.898098);
final start = Point<double>(0.0, 0.0);
final end = Point<double>(10.0, 5.0);

final dist = GoogleMapsUtils.poly.distanceToLine(p, start, end);
// 3675538.019968191 meters
```

### Compute Area

```dart
final area = GoogleMapsUtils.spherical.computeArea(polygon);
// Area in square meters

final signedArea = GoogleMapsUtils.spherical.computeSignedArea(polygon);
// Positive = counter-clockwise, negative = clockwise
```

### Bounding Boxes

```dart
// Create bounds from a center point + radius
// Returns a LatLngBounds record: (ne: Point<double>, sw: Point<double>)
final bounds = GoogleMapsUtils.spherical.toBounds(-23.55, -46.63, 1000.0);
print(bounds.ne); // north-east corner
print(bounds.sw); // south-west corner

// Create bounds from a list of points
final bounds = GoogleMapsUtils.spherical.toBoundsFromPoints(path);

// Get the center of a bounds
final center = GoogleMapsUtils.spherical.centerFromLatLngBounds(bounds);

// Split bounds into a grid (division² sub-bounds)
final tiles = GoogleMapsUtils.spherical.toSubBounds(bounds, division: 3);
// Returns 9 sub-bounds
```

### Closed Polygon Check

```dart
GoogleMapsUtils.poly.isClosedPolygon(polygon); // true if first == last point
```

### Custom Tolerance (On-Path / On-Edge)

```dart
// Default tolerance is 0.1 meters. Use custom tolerance for proximity:
final isNear = GoogleMapsUtils.poly.isLocationOnPathTolerance(
  point, polyline, true, 50.0, // 50 meters tolerance
);

// Same for edges:
final isNearEdge = GoogleMapsUtils.poly.isLocationOnEdgeTolerance(
  point, polygon, true, 50.0,
);

// With index:
final idx = GoogleMapsUtils.poly.locationIndexOnPathTolerance(
  point, polyline, true, 50.0,
);
```

### Math Helpers (Low-Level)

```dart
// Clamp a value to a range
GoogleMapsUtils.math.clamp(15.0, 0.0, 10.0); // 10.0

// Wrap a value into [min, max) — useful for longitude
GoogleMapsUtils.math.wrap(190.0, -180.0, 180.0); // -170.0

// Non-negative modulo
GoogleMapsUtils.math.mod(-1.0, 360.0); // 359.0

// Mercator projection (radians)
final y = GoogleMapsUtils.math.mercator(latInRadians);
final lat = GoogleMapsUtils.math.inverseMercator(y);

// Haversine / inverse haversine
final h = GoogleMapsUtils.math.hav(angleInRadians);
final angle = GoogleMapsUtils.math.arcHav(h);

// Haversine distance on unit sphere
final havDist = GoogleMapsUtils.math.havDistance(lat1, lat2, dLng);
```

### Degree / Radian Conversions

```dart
GoogleMapsUtils.spherical.toRadians(90.0);  // 1.5707963... (π/2)
GoogleMapsUtils.spherical.toDegrees(pi);    // 180.0
```

---

## 📋 Full Example

See [`example/example.dart`](example/example.dart) for a complete runnable demo covering all features.

<details>
<summary>Click to expand example output</summary>

```
Heading: 26.302486345342523 degrees
Angle: 0.19493500057547358 radians
Distance to Line: 3675538.019968191 meters
path size length: 17
simplified path: wjiaFz`hgQcjIke\t{d@|aOutJz|UokC}xWomJdjM
path size simplified length: 6
Distance: 1241932.5985192063 meters
Distance: 1066.7243813716539 meters of polygon
point is inside polygon?: true
```

</details>

---

## 📐 API Reference

### `GoogleMapsUtils.spherical`

| Method | Description |
|--------|-------------|
| `computeDistanceBetween(from, to)` | Distance in meters between two points |
| `computeHeading(from, to)` | Heading in degrees \[-180, 180) |
| `computeAngleBetween(from, to)` | Angle in radians (unit-sphere distance) |
| `computeOffset(from, distance, heading)` | Point at a given distance/heading |
| `computeOffsetOrigin(to, distance, heading)` | Inverse of `computeOffset` |
| `interpolate(from, to, fraction)` | Spherical interpolation (SLERP) |
| `computeLength(path)` | Total path length in meters |
| `computeArea(path)` | Enclosed area in m² |
| `computeSignedArea(path)` | Signed area (determines winding) |
| `distanceRadians(lat1, lng1, lat2, lng2)` | Arc distance on unit sphere (radians in, radians out) |
| `toBounds(lat, lng, radius)` | Bounding box from center + radius |
| `toBoundsFromPoints(points)` | Bounding box from point list |
| `centerFromLatLngBounds(bounds)` | Center point of a bounds |
| `toSubBounds(bounds, division:)` | Split bounds into a grid |
| `getCardinal(angle)` | 16-point compass abbreviation |
| `toRadians(deg)` | Degrees to radians |
| `toDegrees(rad)` | Radians to degrees |

### `GoogleMapsUtils.poly`

| Method | Description |
|--------|-------------|
| `decode(encoded)` | Decode a polyline string to points |
| `encode(path)` | Encode points to a polyline string |
| `simplify(poly, tolerance)` | Douglas-Peucker simplification |
| `containsLocationPoly(point, polygon)` | Spherical point-in-polygon test |
| `isLocationOnPath(point, polyline, geodesic)` | Point-on-polyline test (0.1m tolerance) |
| `isLocationOnPathTolerance(point, polyline, geodesic, tolerance)` | Point-on-polyline with custom tolerance |
| `isLocationOnEdge(point, polygon, geodesic)` | Point-on-polygon-edge test (0.1m tolerance) |
| `isLocationOnEdgeTolerance(point, polygon, geodesic, tolerance)` | Point-on-edge with custom tolerance |
| `locationIndexOnPath(point, polyline, geodesic)` | Segment index or -1 |
| `locationIndexOnPathTolerance(point, poly, geodesic, tolerance)` | Segment index with custom tolerance |
| `locationIndexOnEdgeOrPath(point, poly, closed, geodesic, tolerance)` | Full control: closed/open + tolerance |
| `distanceToLine(point, start, end)` | Distance to a line segment |
| `isClosedPolygon(poly)` | Whether first == last point |

### `GoogleMapsUtils.math`

| Method | Description |
|--------|-------------|
| `earthRadius` | Earth's mean radius in meters (6371008.7714) |
| `clamp(x, low, high)` | Restrict value to range |
| `wrap(n, min, max)` | Wrap into \[min, max) |
| `mod(x, m)` | Non-negative modulo |
| `hav(x)` | Haversine |
| `arcHav(x)` | Inverse haversine |
| `sinFromHav(h)` | sin(\|x\|) given hav(x) |
| `havFromSin(x)` | hav(asin(x)) |
| `sinSumFromHav(x, y)` | sin(arcHav(x) + arcHav(y)) |
| `mercator(lat)` | Latitude (radians) → Mercator Y |
| `inverseMercator(y)` | Mercator Y → latitude (radians) |
| `havDistance(lat1, lat2, dLng)` | Haversine distance on unit sphere |

---

## 📊 Status

**3 of 78** classes from android-maps-utils have been converted — these are the most widely used. Need another class? [Open an issue](https://github.com/KohlsAdrian/google_maps_utils/issues) and I'll prioritize it.

---

## 💚 Sponsor

This package is proudly sponsored by **[bus2.me](https://bus2.me)**.

If this package saves you time, consider supporting its development.

---

## 🔗 Links

- 📦 [pub.dev package](https://pub.dev/packages/google_maps_utils)
- 📖 [API documentation](https://pub.dev/documentation/google_maps_utils/latest/)
- 🐛 [Issue tracker](https://github.com/KohlsAdrian/google_maps_utils/issues)
- 🤖 [android-maps-utils (upstream)](https://github.com/googlemaps/android-maps-utils)
- 💬 [Stack Overflow](https://stackoverflow.com/questions/tagged/android-maps-utils)
- 🦋 [Flutter community request](https://github.com/flutter/flutter/issues/24689)

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes with tests
4. Ensure `dart analyze --fatal-infos` and `dart test` pass
5. Open a Pull Request

---

## 📄 License

This project is licensed under the Apache License 2.0 — see the [LICENSE](LICENSE) file for details.

Original Java source: Copyright 2008, 2013 Google Inc.
