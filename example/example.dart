import 'package:google_maps_utils/google_maps_utils.dart';

void main() {
  LatLng from = LatLng(0.0, 0.0);
  LatLng to = LatLng(10.0, 5.0);
  LatLng randomPoint = LatLng(-23.54545, -23.898098);

  print('Distance: ' +
      SphericalUtils.computeDistanceBetween(from, to).toString());

  print('Heading: ' + SphericalUtils.computeHeading(from, to).toString());

  print('Angle: ' + SphericalUtils.computeAngleBetween(from, to).toString());

  print('Distance to Line: ' +
      PolyUtils.distanceToLine(randomPoint, from, to).toString());

  /// Distance: 1241932.6430813475
  /// Heading: 26.302486345342523
  /// Angle: 0.19493500057547358
  /// Distance to Line: 3675538.1518512294

  /// And Many more
}
