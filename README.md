# An Android Google Maps Utils original java project port for Flutter

A Flutter/Dart project based on Android Google Maps Utils converted/ported to Dart/Flutter
There are many packages that features Google Maps Utils, but none has the 3 main classes
from the sources

# Status: 3 of 78 classes converted

## How it works

This is a project for converting all of the tools from Android Google Maps Utils partially.
Initial commits from the 3 main Classes, MathUtils.java, PolyUtils.java and SphericalUtils.java

The same rules for calculating bounds, distance between two LatLng points and other polyline features are the same as used on Android native Java/Kotlin logic.

All of the methods of current classes available are ported

You can request classes for me to port as an issue, the classes ported are what most people uses

## Feature links for reasearch

Main project: https://github.com/googlemaps/android-maps-utils

Main project questions: https://stackoverflow.com/questions/tagged/android-maps-utils

Fluttert community request: https://github.com/flutter/flutter/issues/24689

## Example *.dart

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


        /// Example by: https://github.com/nicolascav
        LatLng point = LatLng(-31.623060136389135, -60.68669021129609);

        /// Triangle
        List<LatLng> polygon = [
            LatLng(-31.624115, -60.688734),
            LatLng(-31.624115, -60.684657),
            LatLng(-31.621594, -60.686717),
            LatLng(-31.624115, -60.688734),
        ];

        bool contains =
            PolyUtils.containsLocationPoly(point.latitude, point.longitude, polygon);
        print('point is inside polygon?: $contains');

        /// And Many more
    }


### Result:

###### Distance: 1241932.6430813475 meters
###### Heading: 26.302486345342523 degrees
###### Angle: 0.19493500057547358 degrees
###### Distance to Line: 3675538.1518512294 meters
###### path size length: 17
###### simplified path: wjiaFz`hgQcjIke\t{d@|aOutJz|UokC}xWomJdjM
###### path size simplified length: 6