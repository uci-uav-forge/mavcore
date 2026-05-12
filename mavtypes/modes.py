from enum import IntEnum


class FlightMode(IntEnum):
    UNKNOWN = -1
    STABILIZE = 0
    ACRO = 1
    ALTHOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEMID = 25
    HELI_AUTOROTATE = 26
    AUTO_RTL = 27
    TURTLE = 28


class FlightModePlane(IntEnum):
    UNKNOWN = -1
    MANUAL = 0
    CIRCLE = 1
    STABILIZE = 2
    TRAINING = 3
    ACRO = 4
    FLY_BY_WIRE_A = 5
    FLY_BY_WIRE_B = 6
    CRUISE = 7
    AUTOTUNE = 8
    AUTO = 10
    RTL = 11
    LOITER = 12
    TAKEOFF = 13
    AVOID_ADSB = 14
    GUIDED = 15
    INITIALISING = 16
    QSTABILIZE = 17
    QHOVER = 18
    QLOITER = 19
    QLAND = 20
    QRTL = 21
