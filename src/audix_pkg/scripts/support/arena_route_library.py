#!/usr/bin/env python3

DEFAULT_ROUTE_NAME = 'double_pinch_figure8'

ROUTE_WAYPOINTS = {
    'double_pinch_figure8': [
        (-3.60, 0.00),
        (-2.20, 1.80),
        (1.35, 1.45),
        (3.20, 2.20),
        (2.90, 0.00),
        (1.35, -1.45),
        (3.20, -2.20),
        (-2.20, -1.80),
        (-3.60, 0.00),
    ],
}


def build_route_waypoints(route_name):
    normalized_name = str(route_name).strip().lower()
    return list(ROUTE_WAYPOINTS.get(normalized_name, ROUTE_WAYPOINTS[DEFAULT_ROUTE_NAME]))