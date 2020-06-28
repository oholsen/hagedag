import GPS

def test_gga():
    nmea = "$GNGGA,140416.00,5948.99861,N,01021.67811,E,4,12,0.59,192.9,M,39.4,M,1.0,1405*68"
    gga = GPS.process(nmea)
    assert gga.lat == 59.8166435
    assert gga.lon == 10.361301833333334
    assert gga.fix == 4
    assert gga.alt == 192.9


def test_rmc():
    nmea = "$GNRMC,140417.00,A,5948.99864,N,01021.67811,E,0.068,,250620,,,R,V*0C"
    rmc = GPS.process(nmea)
    assert rmc.speed_knots == 0.068
    assert rmc.course_over_ground is None
    assert rmc.date == "250620"
    assert rmc.mode == "R"
