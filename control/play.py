import logging
import sys
import RobotState
import GPS
from datetime import datetime

def main():
    filename = sys.argv[1]
    for line in open(filename):
        line = line.strip()
        # print(line)
        cols = line.split()
        if len(cols) < 4:
            continue
        t = line[:23].replace(",", ".")
        tt = datetime.fromisoformat(t)
        # print(t, tt, line)
        # continue
        if len(cols) == 5 and cols[3] == "GPS":
            msg = cols[4]
            o = GPS.process(msg)
            if o:
                print(tt, "GPS", msg, o)
        elif len(cols) > 5 and cols[3] == "ROBOT":
            msg = " ".join(cols[4:])
            o = RobotState.process(msg)
            if o:
                print(tt, "ROBOT", msg, o)


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        filename="record.log",
        format='%(asctime)s %(levelname)s %(message)s',
    )
    main()
