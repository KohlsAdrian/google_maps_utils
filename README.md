# google_maps_utils - An Android Google Maps Utils java project port

A Flutter/Dart project based on Android Google Maps Utils converted/ported to Dart/Flutter
There are many packages that features Google Maps Utils, but none has the 3 main classes
from the sources

## How it works

This is a project for converting all of the tools from Android Google Maps Utils partially.
Initial commits from the 3 main Classes, MathUtils.java, PolyUtils.java and SphericalUtils.java

The same rules for calculating bounds, distance between two LatLng points and other polyline features are the same as used on Android native Java/Kotlin logic.

All of the methods of current classes available are ported

## Feature links for reasearch

Main project: https://github.com/googlemaps/android-maps-utils
Main project questions: https://stackoverflow.com/questions/tagged/android-maps-utils
Fluttert community request: https://github.com/flutter/flutter/issues/24689

# spherical_utils.dart
    Working 100% - Used in production and working as expected
# math_utils.dart 
    Working 100% - Used in production and working as expected
# poly_utils.dart
    May be broken

## warnings:

# poly_utils.dart 
    _encode
        # Charcode conversion and StringBuffer writing may not be written correctly
    simplify
        # Dart does not have a class for Stacking objects, was used a pub dart for stack that is not in stable state
    decode
        # Charcode conversion may not be written correctly
