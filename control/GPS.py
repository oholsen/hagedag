from typing import Optional
from functools import reduce
import json
import utm


# http://navspark.mybigcommerce.com/content/NMEA_Format_v0.1.pdf
# online decode with map: https://rl.se/gprmc

def to_float(s: str) -> Optional[float]:
    try:
        return float(s)
    except ValueError:
        return None

def to_int(s: str) -> Optional[int]:
    try:
        return int(s)
    except ValueError:
        return None


def parse_lat(dms: str, a: str) -> float:
    d = int(dms[0:2])
    m = float(dms[2:])
    lat = d + m / 60.0
    if a == 'S':
        lat = -lat
    return lat


def parse_time(hms: str) -> float:
    h = int(hms[0:2])
    m = int(hms[2:4])
    s = float(hms[4:])
    return ((h * 60 + m) * 60) + s


def parse_lon(dms: str, a: str) -> float:
    d = int(dms[0:3])
    m = float(dms[3:])
    lon = d + m / 60.0
    if a == 'W':
        lon = -lon
    return lon


def _checksum(message: str) -> str:
    cs = reduce(lambda cs, c: cs ^ ord(c), message, 0)
    return f"{cs:02X}"


def _check_checksum(message: str) -> bool:
    assert message[0] == "$"
    message = message[1:]
    message, checksum = message.split('*')
    return _checksum(message) == checksum


class GPS(object):
    pass


class GGA(GPS):
    def __init__(self, segments):
        self.time = parse_time(segments[1])
        self.lat = parse_lat(segments[2], segments[3])
        self.lon = parse_lon(segments[4], segments[5])
        self.fix = to_int(segments[6])
        self.sats = to_int(segments[7])
        self.hdop = to_float(segments[8])
        self.alt = to_float(segments[9])


class VTG(GPS):
    def __init__(self, segments):
        self.course = to_float(segments[1]) # True north
        self.speed = to_float(segments[7]) # km/h


class RMC(GPS):
    def __init__(self, segments):
        self.time = parse_time(segments[1])
        self.status = segments[2]
        self.lat = parse_lat(segments[3], segments[4])
        self.lon = parse_lon(segments[5], segments[6])
        self.speed_knots = to_float(segments[8])
        self.course_over_ground = to_float(segments[9])
        self.date = segments[9] # "ddmmyy"
        self.mode = segments[10] # "R" for RTK


class Ignore(GPS):
    def __init__(self, segments):
        self.segments = segments


_sentences = {
    "GNGGA": GGA,
    "GNVTG": VTG,
    "GNRMC": RMC,
}


def process(line):
    line = line.strip()
    assert _check_checksum(line)
    segments = line.split(',')
    cmd = segments[0]
    assert cmd[0] == "$"
    cmd = cmd[1:]
    sentence = _sentences.get(cmd) # , Record)
    if sentence:
        return sentence(segments)


class UTM(object):
    def __init__(self, lat, lon):
        self.easting, self.northing, self.zone_number, self.zone_letter = utm.from_latlon(lat, lon)

    def diff(self, u0):
        return self.easting - u0.easting, self.northing - u0.northing


class LatLon(object):
    def __init__(self, lat, lon):
        self.lat = lat
        self.lon = lon


p0 = LatLon(59.81665870, 10.36129810)
u0 = UTM(p0.lat, p0.lon)


def main():
    
    import time
    from simulation.Plotting import Plot
    plot = Plot()

    vtg = None # vtg reported before gga on the same fix

    for line in open("data/gps.log"):
        #print(line)
        o = process(line)
        if o is not None:
            # c = type(o).__name__
            # print(c, json.dumps(o.__dict__))
            if isinstance(o, GGA):
                u = UTM(o.lat, o.lon)
                # print(o.time, o.lat, o.lon, o.alt)
                deasting = u.easting - u0.easting
                dnorthing = u.northing - u0.northing
                # print(o.time, deasting, dnorthing, o.alt)
                if vtg is not None: # and vtg.course is not None:
                    print(o.time, deasting, dnorthing, o.alt, vtg.course, vtg.speed)
                    if vtg.course is not None:
                        plot.plot(deasting, dnorthing, vtg.speed, vtg.course)
                        time.sleep(0.3)
            elif isinstance(o, VTG):
                vtg = o
    

if __name__ == "__main__":
    main()