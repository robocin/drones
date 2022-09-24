"""Created by felipe-nunes on 23/09/2022
"""
from rcpilot.environment import Arena
from sympy.geometry import Point3D


class Base:
    _base_type = Arena.UNKNOWN_BASE
    _is_visited = False
    _position = None

    def __init__(self, arena_type=None):
        if arena_type is not None:
            self._base_type = arena_type

    # Region getters
    @property
    def base_type(self):
        return self._base_type

    @property
    def is_visited(self):
        return self._is_visited

    @property
    def position(self):
        return self._position

    # Region setters
    @base_type.setter
    def base_type(self, base_type: Arena):
        self._base_type = base_type

    @is_visited.setter
    def is_visited(self, is_visited: bool):
        self._is_visited = is_visited

    @position.setter
    def position(self, position: Point3D):
        self._position = position
