import 'package:google_maps_utils/google_maps_utils.dart';
import 'package:poly/poly.dart';

void main() async {
  Point from = Point(0.0, 0.0);
  Point to = Point(10.0, 5.0);
  Point randomPoint = Point(-23.54545, -23.898098);

  double distance = SphericalUtils.computeDistanceBetween(from, to);
  print('Distance: $distance meters');

  double heading = SphericalUtils.computeHeading(from, to);
  print('Heading: $heading degrees');

  double angle = SphericalUtils.computeAngleBetween(from, to);
  print('Angle: $angle degrees');

  double distanceToAB = PolyUtils.distanceToLine(randomPoint, from, to);
  print('Distance to Line: $distanceToAB meters');

  /// Distance: 1241932.6430813475
  /// Heading: 26.302486345342523
  /// Angle: 0.19493500057547358
  /// Distance to Line: 3675538.1518512294

  /// See grid path on: https://developers.google.com/maps/documentation/utilities/polylinealgorithm

  List<Point> path = PolyUtils.decode(
      'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD');

  print('path size length: ${path.length}');

  List<Point> simplifiedPath = PolyUtils.simplify(path, 5000);
  String simplifiedPathEncoded = PolyUtils.encode(simplifiedPath);

  print('simplified path: $simplifiedPathEncoded');
  print('path size simplified length: ${simplifiedPath.length}');

  /// Example by: https://github.com/nicolascav
  Point point = Point(-31.623060136389135, -60.68669021129609);

  /// Triangle
  List<Point> polygon = [
    Point(-31.624115, -60.688734),
    Point(-31.624115, -60.684657),
    Point(-31.621594, -60.686717),
    Point(-31.624115, -60.688734),
  ];

  bool contains = PolyUtils.containsLocationPoly(point, polygon);
  print('point is inside polygon?: $contains');

  try {
    String osrmShape = 'ohr~Fn{mvOve@vqA_GmlBeJqwDkEceI~qDwIzvDpqR';
    final decoded = PolyUtils.decode(osrmShape);
    print('shape decoded: $decoded');
  } catch (e) {
    print(e);
  }

  /// And Many more
}
