"""Central configuration for restaurant maps, table targets and dynamic obstacles.

Use the environment variable MAP_ID (or RESTAURANT_MAP) to select the map:
    MAP_ID=map1
    MAP_ID=map2

The rest of the controllers should not hard-code table coordinates anymore.
They should call get_map_config() and use the returned dictionaries.
"""

from copy import deepcopy
import os
from copy import deepcopy
import os

DEFAULT_MAP_ID = "map1"   # "map1" or "map2"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _reach_points_from_tables(tables, offset=0.0984):
    """Create 5 service points per table: centre + four approach points."""
    return {
        table_id: [
            (x, y),
            (x, y + offset),
            (x, y - offset),
            (x + offset, y),
            (x - offset, y),
        ]
        for table_id, (x, y) in tables.items()
    }


def _normalise_map_id(map_id=None):
    if map_id is None:
        map_id = (
            os.environ.get("MAP_ID")
            or os.environ.get("RESTAURANT_MAP")
            or DEFAULT_MAP_ID
        )

    map_id = str(map_id).strip()

    if map_id == "":
        map_id = DEFAULT_MAP_ID

    key = map_id.lower().replace("_", "").replace("-", "")

    aliases = {
        "1": "map1",
        "m1": "map1",
        "map1": "map1",
        "baseline": "map1",

        "2": "map2",
        "m2": "map2",
        "map2": "map2",
        "lshape": "map2",
        "lshaped": "map2",
    }

    return aliases.get(key, key)

# ---------------------------------------------------------------------------
# Map 1 — baseline rectangular restaurant
# ---------------------------------------------------------------------------

TABLES_1 = {
    "T1": (-0.432, -0.312),
    "T2": (-0.168, -0.120),
    "T3": (-0.408, 0.204),
    "T4": (0.396, -0.252),
    "T5": (0.144, 0.084),
    "T6": (0.432, 0.336),
}

TABLE_REACH_POINTS_1 = _reach_points_from_tables(TABLES_1, offset=0.17)

MAP1_CHAIR_SIZE = 0.0504
MAP1_CHAIR_POINTS = _reach_points_from_tables(TABLES_1, offset=0.0984)
MAP1_CHAIR_RECTS = [
    (x, y, MAP1_CHAIR_SIZE, MAP1_CHAIR_SIZE)
    for points in MAP1_CHAIR_POINTS.values()
    for x, y in points[1:]
]

MAP1_RECTS = [
    # tables
    (-0.432, -0.312, 0.126, 0.126),
    (-0.168, -0.120, 0.126, 0.126),
    (-0.408, 0.204, 0.126, 0.126),
    (0.396, -0.252, 0.126, 0.126),
    (0.144, 0.084, 0.126, 0.126),
    (0.432, 0.336, 0.126, 0.126),
    # chairs
    *MAP1_CHAIR_RECTS,
    # counter / robot base
    (0.0, -0.486, 0.384, 0.066),
]

MAP1_CIRCLES = [
    (-0.576, 0.480, 0.042),
    (0.576, 0.480, 0.042),
    (-0.576, -0.432, 0.042),
    (0.576, -0.432, 0.042),
]

MAP1_DYNAMIC_CIRCUITS = {
    "top_corridor": [
        (-0.46, 0.40),
        (-0.25, 0.40),
        (-0.05, 0.40),
        (0.15, 0.40),
        (0.28, 0.40),
    ],
    "around_table": [
        (-0.04, 0.25),
        (0.33, 0.25),
        (0.33, -0.10),
        (-0.02, -0.08),
    ],
    "base_area": [
        (0.52, -0.41),
        (0.50, -0.43),
        (0.44, -0.43),
        (0.36, -0.43),
        (0.28, -0.42),
        (0.20, -0.40),
        (0.10, -0.395),
        (0.00, -0.395),
        (-0.12, -0.395),
        (-0.20, -0.400),
        (-0.26, -0.430),
        (-0.32, -0.465),
        (-0.40, -0.480),
        (-0.48, -0.480),
        (-0.54, -0.455),
        (-0.55, -0.425),
        (-0.54, -0.455),
        (-0.48, -0.480),
        (-0.40, -0.480),
        (-0.32, -0.465),
        (-0.26, -0.430),
        (-0.20, -0.400),
        (-0.12, -0.395),
        (0.00, -0.395),
        (0.10, -0.395),
        (0.20, -0.40),
        (0.28, -0.42),
        (0.36, -0.43),
        (0.44, -0.43),
        (0.50, -0.43),
    ],
}


# ---------------------------------------------------------------------------
# Map 2 — alternative L-shaped layout
# Coordinates taken from restaurant_mapa2.wbt / mapa2.obj.
# ---------------------------------------------------------------------------

TABLES_2 = {
    "T1": (0.454, 0.083),
    "T2": (0.090, -0.053),
    "T3": (-0.420, 0.312),
    "T4": (-0.030, 0.336),
    "T5": (0.301, 0.522),
    "T6": (-0.240, 0.630),
}

TABLE_REACH_POINTS_2 = _reach_points_from_tables(TABLES_2, offset=0.14)

MAP2_CHAIR_SEATS = [
    (0.454, 0.177), (0.454, -0.011), (0.548, 0.083), (0.360, 0.083),
    (0.090, 0.041), (0.090, -0.147), (0.184, -0.053), (-0.004, -0.053),
    (-0.420, 0.406), (-0.420, 0.218), (-0.326, 0.312), (-0.514, 0.312),
    (-0.030, 0.430), (-0.030, 0.242), (0.064, 0.336), (-0.124, 0.336),
    (0.301, 0.616), (0.301, 0.428), (0.395, 0.522), (0.207, 0.522),
    (-0.240, 0.694), (-0.240, 0.530), (-0.146, 0.630), (-0.334, 0.630),
]

MAP2_CHAIR_BACKS_HORIZONTAL = [
    (0.454, 0.200), (0.454, -0.034),
    (0.090, 0.064), (0.090, -0.170),
    (-0.420, 0.429), (-0.420, 0.195),
    (-0.030, 0.453), (-0.030, 0.219),
    (0.301, 0.639), (0.301, 0.405),
    (-0.240, 0.717), (-0.240, 0.510),
]

MAP2_CHAIR_BACKS_VERTICAL = [
    (0.571, 0.083), (0.337, 0.083),
    (0.207, -0.053), (-0.027, -0.053),
    (-0.303, 0.312), (-0.537, 0.312),
    (0.087, 0.336), (-0.147, 0.336),
    (0.418, 0.522), (0.184, 0.522),
    (-0.123, 0.630), (-0.357, 0.630),
]

MAP2_CHAIR_RECTS = [
    *[(x, y, 0.048, 0.048) for x, y in MAP2_CHAIR_SEATS],
    *[(x, y, 0.048, 0.008) for x, y in MAP2_CHAIR_BACKS_HORIZONTAL],
    *[(x, y, 0.008, 0.048) for x, y in MAP2_CHAIR_BACKS_VERTICAL],
]

MAP2_RECTS = [
    # external and internal walls — synced with restaurant_mapa2.wbt
    (0.000, 0.789, 1.236, 0.018),
    (-0.609, 0.4305, 0.018, 0.717),
    (0.609, 0.210, 0.018, 1.176),
    (0.255, -0.369, 0.726, 0.018),
    (-0.345, 0.081, 0.510, 0.018),
    (-0.099, -0.135, 0.018, 0.450),

    # tables
    (0.454, 0.083, 0.126, 0.126),
    (0.090, -0.053, 0.126, 0.126),
    (-0.420, 0.312, 0.126, 0.126),
    (-0.030, 0.336, 0.126, 0.126),
    (0.301, 0.522, 0.126, 0.126),
    (-0.240, 0.630, 0.126, 0.126),

    # chairs
    *MAP2_CHAIR_RECTS,

    # counter / robot base
    (0.300, -0.306, 0.336, 0.066),
]

MAP2_CIRCLES = [
    (0.540, -0.300, 0.041),
    (0.540, 0.696, 0.041),
    (0.066, 0.604, 0.041),
    (-0.520, 0.621, 0.041),
]

MAP2_DYNAMIC_CIRCUITS = {
    "upper_corridor": [
        (-0.10, 0.720),
        (0.16, 0.720),
    ],

    "central_passage": [
        (0.10, 0.170),
        (0.28, 0.170),
        (0.28, 0.290),
        (0.10, 0.290),
    ],

    "right_branch": [
        (0.525, 0.300),
        (0.525, 0.500),
    ],
}

# ---------------------------------------------------------------------------
# Shared dynamic-obstacle timing and obstacle profiles
# ---------------------------------------------------------------------------

DYNAMIC_WINDOWS = {
    "MOVING_PERSON_1": [(0.0, 20.0), (60.0, 85.0)],
    "MOVING_PERSON_2": [(20.0, 40.0), (60.0, 85.0)],
    "MOVING_PERSON_3": [(40.0, 60.0), (60.0, 85.0)],
}

DYNAMIC_OBSTACLES = [
    {
        "def_name": "MOVING_PERSON_1",
        "lap_time": 14.0,
        "z_height": 0.070,
        "circuit_offset": 0,
        "active_windows": DYNAMIC_WINDOWS["MOVING_PERSON_1"],
    },
    {
        "def_name": "MOVING_PERSON_2",
        "lap_time": 9.0,
        "z_height": 0.055,
        "circuit_offset": 1,
        "active_windows": DYNAMIC_WINDOWS["MOVING_PERSON_2"],
    },
    {
        "def_name": "MOVING_PERSON_3",
        "lap_time": 10.0,
        "z_height": 0.045,
        "circuit_offset": 2,
        "active_windows": DYNAMIC_WINDOWS["MOVING_PERSON_3"],
    },
]


MAP_CONFIGS = {
    "map1": {
        "id": "map1",
        "name": "baseline rectangular restaurant",
        "table_ids": list(TABLES_1.keys()),
        "tables": TABLES_1,
        "table_reach_points": TABLE_REACH_POINTS_1,
        "base_pos": (0.000, -0.390),
        "table_arrival_radius": 0.150,
        "base_arrival_radius": 0.050,
        "known_map": {
            "resolution": 0.03,
            "origin_x": -0.66,
            "origin_y": -0.57,
            "width": 44,
            "height": 38,
            "floor_rectangles": [(0.0, 0.0, 1.32, 1.14)],
            "rectangles": MAP1_RECTS,
            "circles": MAP1_CIRCLES,
            "inflate_radius_cells": 1,
        },
        "dynamic_obstacles": {
            "scenario_period": 95.0,
            "circuits": MAP1_DYNAMIC_CIRCUITS,
            "obstacles": DYNAMIC_OBSTACLES,
        },
    },
    "map2": {
        "id": "map2",
        "name": "alternative L-shaped restaurant",
        "table_ids": list(TABLES_2.keys()),
        "tables": TABLES_2,
        "table_reach_points": TABLE_REACH_POINTS_2,
        "base_pos": (0.300, -0.220),
        "table_arrival_radius": 0.150,
        "base_arrival_radius": 0.060,
        "known_map": {
            "resolution": 0.03,
            "origin_x": -0.66,
            "origin_y": -0.42,
            "width": 50,
            "height": 43,
            # Two rectangles model the L-shaped walkable floor. Everything outside
            # them is treated as occupied in the known map, so A* does not plan
            # through the missing part of the L.
            "floor_rectangles": [
                (0.000, 0.435, 1.200, 0.690),
    (           0.255, -0.135, 0.690, 0.450),
            ],
            "rectangles": MAP2_RECTS,
            "circles": MAP2_CIRCLES,
            "inflate_radius_cells": 1,
        },
        "dynamic_obstacles": {
            "scenario_period": 95.0,
            "circuits": MAP2_DYNAMIC_CIRCUITS,
            "obstacles": DYNAMIC_OBSTACLES,
        },
    },
}


def get_map_config(map_id=None):
    selected = _normalise_map_id(map_id)

    if selected not in MAP_CONFIGS:
        valid = ", ".join(sorted(MAP_CONFIGS))
        raise ValueError(
            f"Unknown MAP_ID={selected!r}. "
            f"Original value={map_id!r}. "
            f"Valid values: {valid}"
        )

    return deepcopy(MAP_CONFIGS[selected])


def get_table_ids(map_id=None):
    return get_map_config(map_id)["table_ids"]


def get_tables(map_id=None):
    return get_map_config(map_id)["tables"]


def get_table_reach_points(map_id=None):
    return get_map_config(map_id)["table_reach_points"]


def get_base_pos(map_id=None):
    return get_map_config(map_id)["base_pos"]
