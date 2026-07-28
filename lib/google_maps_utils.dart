/// Google Maps Utils — a port of the android-maps-utils library.
///
/// Use [GoogleMapsUtils] as the unified entry point:
/// ```dart
/// GoogleMapsUtils.spherical.computeDistanceBetween(from, to);
/// GoogleMapsUtils.poly.decode(encodedPath);
/// GoogleMapsUtils.math.wrap(190.0, -180.0, 180.0);
/// ```
library google_maps_utils;

export 'google_maps_utils_facade.dart';
export 'google_maps_utils_lat_lng_bounds.dart';
