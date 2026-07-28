# Changelog

## 4.0.0

* **BREAKING:** removed `SphericalUtils.computeDistanceFromListOfPoints`.
  Use `SphericalUtils.computeLength` instead — it has identical behaviour.
* **BREAKING:** removed `SphericalUtils.polarTriangleArea`. This was an
  implementation detail that was accidentally public. Use `computeArea` or
  `computeSignedArea` for area calculations.
* **BREAKING:** renamed `GMULatLngBounds` to `GoogleMapsUtilsLatLngBounds`.
* **BREAKING:** SDK constraint bumped from `>=2.12.0 <4.0.0` to
  `>=3.0.0 <4.0.0`.
* **New:** added `GoogleMapsUtils` facade class with `.math`, `.poly`, and
  `.spherical` getters for a unified, Flutter-idiomatic access pattern:
  ```dart
  GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
  GoogleMapsUtils.poly.decode(encoded);
  GoogleMapsUtils.math.wrap(190, -180, 180);
  ```
  The original `SphericalUtils`, `PolyUtils`, and `MathUtils` classes remain
  available for direct static access.
* **No longer mutates input:** `PolyUtils.simplify` now works on an internal
  copy instead of calling `removeAt`/`add` on the caller's list.
* **Added** `GMULatLngBounds.==`, `hashCode`, and `toString` for easier
  debugging and testing.
* **Improved** `ArgumentError` is now thrown instead of plain `Exception` in
  `PolyUtils.simplify` and `SphericalUtils.toBoundsFromPoints`.
* **Style:** `MathUtils.earthRadius` is now a compile-time `const` instead of
  `static final`. Default tolerance in `PolyUtils` is also `const`.
* **CI:** updated GitHub Actions workflow to `checkout@v4`, `setup-dart@v1`
  (stable), enabled `dart format` check, and `--fatal-infos` analysis.
* **Pub score:** added `topics` field to pubspec for better discoverability.
* **Docs:** fixed typos in README, added dartdoc links to cross-referenced
  classes, improved doc comments across all source files.
* **Internal:** replaced `List.generate` with `List.filled` where appropriate,
  replaced `.forEach` with `for-in` loops, removed redundant `size()` method
  from internal `Stack` class.
* **Assets:** added `maps-icon.svg` repo icon (generated with Kiro).

_Audited and fixed with Kiro._

## 3.0.0

* **BREAKING:** the public API is now typed with `double` instead of `num`.
  Coordinates use `Point<double>` (so methods take `Point<double>` /
  `List<Point<double>>` and scalar parameters like `distance`, `heading`,
  `tolerance`, `fraction` and `radius` are `double`); all numeric return types
  are now `double` (or `int` for indices). Callers using `List<Point>`,
  `Point<num>` or integer-coordinate points must switch to `Point<double>`
  (e.g. `Point(0.0, 0.0)`). Behaviour and results are unchanged.
* **Fixed** `PolyUtils.isLocationOnEdge`, `isLocationOnPath` and
  `locationIndexOnPath`/`locationIndexOnEdgeOrPath` always reporting "not on
  path" in geodesic mode (even for a point lying exactly on the line). Three
  boolean returns in the private `_isOnSegmentGC` helper were inverted during the
  original Java port; they now match the canonical `android-maps-utils` source.
* **Fixed & upgraded** `PolyUtils.containsLocationPoly`. It returned wrong
  results on shapes with multiple vertices when the test point shared a latitude
  with a vertex ([issue #11](https://github.com/KohlsAdrian/google_maps_utils/issues/11)).
  It is now a faithful spherical port of android-maps-utils'
  `PolyUtil.containsLocation` (verified identical in v2.3.0 and v4.5.1), so it is
  also correct for polygons that **cross the antimeridian (±180°)** or **enclose
  a pole** — cases the old planar ray-cast could not handle. Added an optional
  `geodesic` parameter (great-circle vs Rhumb edges; defaults to `false`, the
  closest match to prior behaviour). Verified against an independent 3D
  winding-number oracle across 200k points. A point on a vertex is inside; the
  result for a point exactly on an edge is now undefined (it was a fragile,
  float-exact check before).
* **Performance:** `PolyUtils.decode` no longer re-splits the entire encoded
  string on every character (O(n²) → O(n)); `encode` internals simplified. Output
  is unchanged.
* **Docs:** corrected stale `Distance` and `Distance to Line` values in the
  README and example (they predated the `earthRadius` precision update) and fixed
  a malformed code fence.
* **Tests:** added regression tests for the geodesic on-path/on-edge bug plus
  broad coverage for previously-untested functions.
* **Pub score:** resolved all `package:lints/core` analysis issues (return-type
  and dangling-doc-comment lints), eliminated every dartdoc warning, documented
  the remaining public members, and added `analysis_options.yaml`, a `lints`
  dev-dependency, a `repository` field and a refreshed package description.
* **Style:** local variables now use `final name = …` (inferred type) when never
  reassigned and an explicit `Type name = …` (no `var`) when reassigned; enabled
  `prefer_final_locals`.
* **Funding:** added a `funding:` entry to the pubspec, a `.github/FUNDING.yml`,
  and a README sponsor section, all pointing to [bus2.me](https://bus2.me).

_Audited and fixed with Claude Opus 4.8 (Ultracode — xhigh + workflows)._ _Sponsored by [bus2.me](https://bus2.me)_

## 2.0.0+1

* Fixed `toSubBounds`

## 2.0.0+0

* BREAKING CHANGE

 - Changed folder files structure
 - Replaced all `int` and `double` by `num`
 
* Added `toSubBounds`

## 1.5.1+0

* Fixed import directive

* Updated `earthRadius` to more precise value

## 1.5.0+0

* Calculate route distance
## 1.4.0+1

* nnbd hotfix for pub.dev dartanalyzer

## 1.4.0

* nnbd migration

## 1.3.0+1

* dartfmt -w .

## 1.3.0

* Added toBoundsFromPoints (Request by <https://github.com/adam-ashored>)

* Added centerFromLatLngBounds

* Fixed some docs

## 1.2.2+1

Fixed: 'List' is deprecated and shouldn't be used.

Updated poly stack package


## 1.2.2+0

Fixed changelog order for new releases, left unordered for old releases

Breaking changes:

- Renamed LatLngBounds to GMULatLngBounds, prevent using prefix import while using google_maps_flutter

- MathUtils, PolyUtils and SphericalUtils are no longer instantiable

## 1.2.1+1

Added all cardinal directions
Removed PT-BR exclusive function translation from cardinals

## 1.2.1+0

Fixed Poly simplify, merge PR thanks to <https://github.com/0oL>

## 1.2.0+3

Replaced LatLng for Point class, kept LatLngBounds class

## 1.2.0+2

Fixed Readme

Added: <https://pub.dev/packages/poly> package for replacing Google's containsLocation function for compiler reasons, google java code could not behave as expected, replacing by new algorithm from Poly package fixed the issue, thanks <https://github.com/nicolascav> for the feedback.

Added <https://github.com/nicolascav> code as example on example.dart

## 1.2.0+1

## 1.2.0+0

### Old realeases

## 1.0.0+1

Added SphericalUtils and MathUtils

## 1.0.0+2

Added Readme, License and PolyUtils

## 1.0.0+3

Simplified and shorted some codes
Fixed Readme

## 1.1.0+1

Added Changelog and formated all files, updated stack package from ^0.0.1 to ^0.1.0
Removed google maps package, replaced for created Lightweight Point and LatLngBounds classes (lat_lng.dart)
Fixed docs in all classes
Added example.dart

## 1.1.0+2

Fixed description

## 1.1.0+3

Fixed description

## 1.1.1+0

Fixed poly_utils.dart, all 3 functions are working as expected
Removed unnecessary dependencies
Updated example.dart
Added example.dart in Readme

## 1.1.1+1

## 1.1.1+2

## 1.1.1+3

Fixed dart.pub health:
DO use curly braces for all flow control structures

## 1.1.1+4

Fixed LatLngBounds variable names
Fixed toBounds method, variables in return were inverted
