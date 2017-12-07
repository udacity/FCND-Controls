# -*- coding: utf-8 -*-
"""
Helper functions
"""
import utm
import numpy as np


def global_to_local(global_position, global_home):
    """
    Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position.
    """
    (east_home, north_home, _, _) = utm.from_latlon(global_home[1],
                                                    global_home[0])
    (east, north, _, _) = utm.from_latlon(global_position[1],
                                          global_position[0])

    local_position = np.array(
        [north - north_home, east - east_home, -global_position[2]])
    return local_position


def local_to_global(local_position, global_home):
    """
    Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)
    """
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(
        global_home[1], global_home[0])
    (lat, lon) = utm.to_latlon(east_home + local_position[1],
                               north_home + local_position[0], zone_number,
                               zone_letter)

    lla = np.array([lon, lat, -local_position[2]])
    return lla


def ned_to_eun(v):
    """
    Helper function to convert from NED frame to unity frame (EUN).
    """
    north = v[0]
    east = v[1]
    down = v[2]
    if down < 0:
        down = -down
    return np.float32([east, down, north])


# TODO: not 100% sure this is correct, mainly about up
def eun_to_ned(v):
    """
    Helper function to convert from unity frame (EUN) to NED frame.
    """
    east = v[0]
    up = v[1]
    north = v[2]
    return np.float32([north, east, -up])

