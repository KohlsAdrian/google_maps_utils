/// Lightweight LatLng class
class LatLng {
  final double latitude;
  final double longitude;

  LatLng(this.latitude, this.longitude);
}

/// Lightweight LatLngBounds class
class LatLngBounds {
  final LatLng northEast;
  final LatLng southWest;

  LatLngBounds(this.northEast, this.southWest);
}
