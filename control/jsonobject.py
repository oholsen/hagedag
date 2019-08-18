"""
Utility for simple access to hierarchical objects, without having to test every element on the path for non-null.
"""

import json


class NoneObject:
    """
    All attributes return NoneObject
    """

    def __getattr__(self, _):
        return self

    def __bool__(self):
        return False

    def __str__(self):
        return "NoneObject"


def fromJson(o):
    """
    :param o:
    :return:
    """
    if isinstance(o, list):
        return [fromJson(x) for x in o]
    if isinstance(o, dict):
        return JsonObject(dict((n, fromJson(v)) for n, v in o.items()))
    if o is None:
        return none
    return o


class _Encoder(json.JSONEncoder):
    """
    JSON encoder of JsonObject
    """

    def default(self, o):  # pylint:disable=method-hidden
        if isinstance(o, JsonObject):
            return o.__dict__
        if isinstance(o, NoneObject):
            return None
        return json.JSONEncoder.default(self, o)


none = NoneObject()
_encoder = _Encoder()


class JsonObject:
    """
    Represent a dictionary as a Python object with attributes.
    """

    def __init__(self, d):
        self.__dict__ = d

    def __getattr__(self, name):
        # Called when not found in dict
        return none

    def json(self):  # pylint:disable=missing-docstring
        return _encoder.encode(self)

    def __str__(self):
        return str(self.__dict__)

    def __repr__(self):
        return repr(self.__dict__)

    def get(self, name):  # pylint:disable=missing-docstring
        return self.__dict__.get(name)


def tests():  # pylint:disable=missing-docstring

    j = {"a": 1, "b": {"c": 3}}
    o = fromJson(j)
    print(j)
    print(o)
    print(o.json())

    def test(x, expected):
        print(x, expected, _encoder.encode(x) == _encoder.encode(expected))

    test(o.a, 1)
    test(o.b, {"c": 3})
    test(o.c, none)
    test(o.b.c, 3)
    test(o.b.d, none)


if __name__ == "__main__":
    tests()
