import 'dart:math';

/// Internal axis-aligned latitude/longitude bounding box model.
///
/// This class is not part of the public API. Users interact with the
/// [GoogleMapsUtilsLatLngBounds] typedef (a Dart record) instead.
class LatLngBoundsModel {
  /// The north-east corner of the bounding box.
  final Point<double> northEast;

  /// The south-west corner of the bounding box.
  final Point<double> southWest;

  /// Creates a bounding box from [northEast] and [southWest] corners.
  LatLngBoundsModel({required this.northEast, required this.southWest});

  @override
  String toString() =>
      'LatLngBoundsModel(ne: (${northEast.x}, ${northEast.y}), sw: (${southWest.x}, ${southWest.y}))';

  @override
  bool operator ==(Object other) =>
      identical(this, other) ||
      other is LatLngBoundsModel &&
          northEast == other.northEast &&
          southWest == other.southWest;

  @override
  int get hashCode => Object.hash(northEast, southWest);
}
