/// Copyright 2013 Google Inc.
/// Licensed under the Apache License, Version 2.0 (the "License");
/// you may not use this file except in compliance with the License.
/// You may obtain a copy of the License at
///      http://www.apache.org/licenses/LICENSE-2.0
/// Unless required by applicable law or agreed to in writing, software
/// distributed under the License is distributed on an "AS IS" BASIS,
/// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
/// See the License for the specific language governing permissions and
/// limitations under the License.

import 'dart:math';

/// Utility functions that are used my both PolyUtil and SphericalUtil.

class MathUtils {
  MathUtils._();

  /// The earth's radius, in meters.
  /// Mean radius as defined by IUGG.

  static final num earthRadius = 6371008.7714;

  /// Restrict x to the range [low, high].

  static num clamp(num x, num low, num high) =>
      x < low ? low : (x > high ? high : x);

  /// Wraps the given value into the inclusive-exclusive interval between min and max.

  /// [n]   The value to wrap.
  ///
  /// [min] The minimum.
  ///
  /// [max] The maximum.
  static num wrap(num n, num min, num max) =>
      (n >= min && n < max) ? n : (mod(n - min, max - min) + min);

  /// Returns the non-negative remainder of x / m.
  ///
  /// [x] The operand.
  ///
  /// [m] The modulus.
  static num mod(num x, num m) => ((x % m) + m) % m;

  /// Returns mercator Y corresponding to latitude.
  ///
  /// See http://en.wikipedia.org/wiki/Mercator_projection .
  static num mercator(num lat) => log(tan(lat * 0.5 + pi / 4));

  /// Returns latitude from mercator Y.
  static num inverseMercator(num y) => 2 * atan(exp(y)) - pi / 2;

  /// Returns haversine(angle-in-radians).
  ///
  /// hav(x) == (1 - cos(x)) / 2 == sin(x / 2)^2.
  static num hav(num x) => pow(sin(x * 0.5), 2);

  /// Computes inverse haversine. Has good numerical stability around 0.
  ///
  /// arcHav(x) == acos(1 - 2 * x) == 2 * asin(sqrt(x)).
  ///
  /// The argument must be in [0, 1], and the result is positive.
  static num arcHav(num x) => 2 * asin(sqrt(x));

  // Given h==hav(x), returns sin(abs(x)).
  static num sinFromHav(num h) => 2 * sqrt(h * (1 - h));

  // Returns hav(asin(x)).
  static num havFromSin(num x) {
    num x2 = x * x;
    return x2 / (1 + sqrt(1 - x2)) * .5;
  }

  // Returns sin(arcHav(x) + arcHav(y)).
  static num sinSumFromHav(num x, num y) {
    num a = sqrt(x * (1 - x));
    num b = sqrt(y * (1 - y));
    return 2 * (a + b - 2 * (a * y + b * x));
  }

  /// Returns hav() of distance from (lat1, lng1) to (lat2, lng2) on the unit sphere.
  static num havDistance(num lat1, num lat2, num dLng) =>
      hav(lat1 - lat2) + hav(dLng) * cos(lat1) * cos(lat2);
}
