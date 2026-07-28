import 'dart:math';

/// A bounding box defined by its north-east and south-west corners.
///
/// This is a lightweight Dart record used as the public API for bounding-box
/// operations. Pass it to any method that requires bounds:
///
/// ```dart
/// final bounds = GoogleMapsUtils.spherical.toBounds(-23.55, -46.63, 1000.0);
/// print(bounds.ne); // north-east corner
/// print(bounds.sw); // south-west corner
/// ```
typedef GoogleMapsUtilsLatLngBounds = ({Point<double> ne, Point<double> sw});
