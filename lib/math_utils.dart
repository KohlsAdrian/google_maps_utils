// Copyright 2013 Google Inc.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//      http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import 'dart:math';

/// Utility functions used by both [PolyUtils] and [SphericalUtils].
class MathUtils {
  MathUtils._();

  /// The earth's mean radius in meters (IUGG).
  static const double earthRadius = 6371008.7714;

  /// Restricts [x] to the range \[[low], [high]\].
  static double clamp(double x, double low, double high) =>
      x < low ? low : (x > high ? high : x);

  /// Wraps [n] into the inclusive-exclusive interval \[[min], [max]).
  static double wrap(double n, double min, double max) =>
      (n >= min && n < max) ? n : (mod(n - min, max - min) + min);

  /// Returns the non-negative remainder of [x] / [m].
  static double mod(double x, double m) => ((x % m) + m) % m;

  /// Returns the Mercator Y coordinate corresponding to [lat] (in radians).
  ///
  /// See http://en.wikipedia.org/wiki/Mercator_projection.
  static double mercator(double lat) => log(tan(lat * 0.5 + pi / 4));

  /// Returns the latitude (in radians) from the Mercator Y coordinate [y].
  static double inverseMercator(double y) => 2 * atan(exp(y)) - pi / 2;

  /// Returns haversine of [x] (angle in radians).
  ///
  /// hav(x) == (1 - cos(x)) / 2 == sin(x / 2)².
  static double hav(double x) {
    final sine = sin(x * 0.5);
    return sine * sine;
  }

  /// Computes the inverse haversine. Numerically stable around 0.
  ///
  /// arcHav(x) == acos(1 - 2x) == 2 × asin(√x).
  ///
  /// The argument must be in `[0, 1]`; the result is non-negative.
  static double arcHav(double x) => 2 * asin(sqrt(x));

  /// Given h == hav(x), returns sin(|x|).
  static double sinFromHav(double h) => 2 * sqrt(h * (1 - h));

  /// Returns hav(asin([x])).
  static double havFromSin(double x) {
    final x2 = x * x;
    return x2 / (1 + sqrt(1 - x2)) * 0.5;
  }

  /// Returns sin(arcHav([x]) + arcHav([y])).
  static double sinSumFromHav(double x, double y) {
    final a = sqrt(x * (1 - x));
    final b = sqrt(y * (1 - y));
    return 2 * (a + b - 2 * (a * y + b * x));
  }

  /// Returns hav() of distance from ([lat1], lng1) to ([lat2], lng2) on the
  /// unit sphere, where [dLng] = lng1 − lng2.
  static double havDistance(double lat1, double lat2, double dLng) =>
      hav(lat1 - lat2) + hav(dLng) * cos(lat1) * cos(lat2);
}
