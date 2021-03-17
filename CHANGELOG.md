# Changelog

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
