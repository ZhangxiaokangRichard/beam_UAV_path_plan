#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""WGS84 ↔ map ENU，算法对齐 ros_mqtt_bridge/geodesy.cpp。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


_PI = math.pi
_A = 6378137.0
_E2 = 6.6943799901413165e-3


@dataclass
class GeodeticPosition:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float


class Geodesy:
    def __init__(self, origin_latitude_deg: float, origin_longitude_deg: float, origin_altitude_m: float):
        self._origin_lat_rad = math.radians(origin_latitude_deg)
        self._origin_lon_rad = math.radians(origin_longitude_deg)
        self._origin_alt_m = origin_altitude_m
        self._origin_x, self._origin_y, self._origin_z = self._geodetic_to_ecef(
            origin_latitude_deg, origin_longitude_deg, origin_altitude_m
        )

    @staticmethod
    def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        radius = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
        x = (radius + alt_m) * math.cos(lat) * math.cos(lon)
        y = (radius + alt_m) * math.cos(lat) * math.sin(lon)
        z = (radius * (1.0 - _E2) + alt_m) * sin_lat
        return x, y, z

    @staticmethod
    def _ecef_to_geodetic(x: float, y: float, z: float) -> GeodeticPosition:
        longitude = math.atan2(y, x)
        p = math.hypot(x, y)
        latitude = math.atan2(z, p * (1.0 - _E2))
        height = 0.0
        for _ in range(8):
            sin_lat = math.sin(latitude)
            radius = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
            height = p / math.cos(latitude) - radius
            latitude = math.atan2(z, p * (1.0 - _E2 * radius / (radius + height)))
        return GeodeticPosition(math.degrees(latitude), math.degrees(longitude), height)

    def to_geodetic(self, x: float, y: float, z: float) -> GeodeticPosition:
        sin_lat = math.sin(self._origin_lat_rad)
        cos_lat = math.cos(self._origin_lat_rad)
        sin_lon = math.sin(self._origin_lon_rad)
        cos_lon = math.cos(self._origin_lon_rad)
        dx = -sin_lon * x - sin_lat * cos_lon * y + cos_lat * cos_lon * z
        dy = cos_lon * x - sin_lat * sin_lon * y + cos_lat * sin_lon * z
        dz = cos_lat * y + sin_lat * z
        return self._ecef_to_geodetic(self._origin_x + dx, self._origin_y + dy, self._origin_z + dz)

    def to_local(self, latitude_deg: float, longitude_deg: float, altitude_m: float) -> Tuple[float, float, float]:
        """WGS84 → map ENU (x=East, y=North, z=Up)。"""
        x, y, z = self._geodetic_to_ecef(latitude_deg, longitude_deg, altitude_m)
        dx = x - self._origin_x
        dy = y - self._origin_y
        dz = z - self._origin_z
        sin_lat = math.sin(self._origin_lat_rad)
        cos_lat = math.cos(self._origin_lat_rad)
        sin_lon = math.sin(self._origin_lon_rad)
        cos_lon = math.cos(self._origin_lon_rad)
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
        return east, north, up
