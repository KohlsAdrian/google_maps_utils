import 'dart:math';

import 'package:google_maps_utils/google_maps_utils.dart';

void main() async {
  final from = Point<double>(0.0, 0.0);
  final to = Point<double>(10.0, 5.0);
  final randomPoint = Point<double>(-23.54545, -23.898098);

  final heading = SphericalUtils.computeHeading(from, to);
  print('Heading: $heading degrees');

  final angle = SphericalUtils.computeAngleBetween(from, to);
  print('Angle: $angle degrees');

  final distanceToAB = PolyUtils.distanceToLine(randomPoint, from, to);
  print('Distance to Line: $distanceToAB meters');

  /// Distance: 1241932.5985192063
  /// Heading: 26.302486345342523
  /// Angle: 0.19493500057547358
  /// Distance to Line: 3675538.019968191

  /// See grid path on: https://developers.google.com/maps/documentation/utilities/polylinealgorithm

  final path = PolyUtils.decode(
      'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD');

  print('path size length: ${path.length}');

  final simplifiedPath = PolyUtils.simplify(path, 5000);
  final simplifiedPathEncoded = PolyUtils.encode(simplifiedPath);

  print('simplified path: $simplifiedPathEncoded');
  print('path size simplified length: ${simplifiedPath.length}');

  final distance = SphericalUtils.computeDistanceBetween(from, to);
  print('Distance: $distance meters');

  /// Triangle
  final polygon = <Point<double>>[
    Point(-31.624115, -60.688734),
    Point(-31.624115, -60.684657),
    Point(-31.621594, -60.686717),
    Point(-31.624115, -60.688734),
  ];

  /// Distance in meters of polygon
  final mDistance = SphericalUtils.computeDistanceFromListOfPoints(polygon);
  print('Distance: $mDistance meters of polygon');

  final point = Point<double>(-31.623060136389135, -60.68669021129609);
  final contains = PolyUtils.containsLocationPoly(point, polygon);
  print('point is inside polygon?: $contains');

  try {
    final osrmShape = 'ohr~Fn{mvOve@vqA_GmlBeJqwDkEceI~qDwIzvDpqR';
    final decoded = PolyUtils.decode(osrmShape);
    print('shape decoded: $decoded');
  } catch (e) {
    print(e);
  }

  try {
    final bounds = GMULatLngBounds(
      northEast: Point(-26.917429376164765, -49.06996585428715),
      southWest: Point(-26.918328620990753, -49.071011915802956),
    );

    final subBounds = SphericalUtils.toSubBounds(bounds);
    print('toSubBounds:[');
    for (final subBound in subBounds) {
      final ne = subBound.northEast;
      final sw = subBound.southWest;
      print('${ne.x},${ne.y}:${sw.x},${sw.y}');
    }
    print(']');
  } catch (e) {
    print(e);
  }

  /// And Many more
}
