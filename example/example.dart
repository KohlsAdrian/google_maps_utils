import 'package:google_maps_utils/google_maps_utils.dart';

void main() {
  LatLng from = LatLng(0.0, 0.0);
  LatLng to = LatLng(10.0, 5.0);
  LatLng randomPoint = LatLng(-23.54545, -23.898098);

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

  List<LatLng> path = PolyUtils.decode(
      'wjiaFz`hgQs}GmmBok@}vX|cOzKvvT`uNutJz|UgqAglAjr@ijBz]opA|Vor@}ViqEokCaiGu|@byAkjAvrMgjDj_A??ey@abD');

  print('path size length: ${path.length}');

  List<LatLng> simplifiedPath = PolyUtils.simplify(path, 5000);
  String simplifiedPathEncoded = PolyUtils.encode(simplifiedPath);

  print('simplified path: $simplifiedPathEncoded');
  print('path size simplified length: ${simplifiedPath.length}');

  /// And Many more
}
