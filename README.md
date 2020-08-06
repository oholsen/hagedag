# hagedag
DIY lawn mower robot

## Testing

Run unit tests:

    poetry run pytest -s

Run simulation:

    poetry run python .\record.py

(Replay log file, should add fail safe detection ++):

    poetry run python .\record.py .\data\burned-motor.log


## TODO

* motor burned (one wheel was tangled in grass):
  * wifi connection delayed - commands and feedback
  * figure out why robot estimate wrongly turned and why it did not recover when moving "right"
  * fix: move control to robot, detect connection delays, ignore delayed information, timestamp from STM32, use timestamp from GPS

* high load on motor detection - possibly integrate and average
  * cut in STM or in control? Control, as will get to know what is going on?
  * STM will stop robot if communication errors, safe to cut in control.
  * different max for instantaneous and average load?
  * Simply stop when above threshold (instantaneous)

* robot stuck detection: anomaly on prediction and GPS
  * measure distance travelled over last 5 say on GPS and compare to integrated speed

* wifi delay detection
    * shorter timeout (1s/2s)
    * embed timeout in command
    * alert in GUI

* stop cutter outside fence?
* cutter stuck detection
* revs sensor on cutter


* slow down near fence
* keep estimating position with odometer only when no GPS fix -> higher uncertainty

* Fence
    * increase area behind garage
    * reduce fence area near stairs

* App robustness
    * restart GPS service when USB unavailable, also ntrip client fails badly? restart service should be ok...
    * properly stop all tasks, also controld connection, from PC app
    * program firmware with heartbeat on commands

Usability:
    * Don't make PC app always be on top: run GUI task in own thread?
    * PC/Web/mobile app framework: pyplot, JS, plotly-dash, ...
    * identify what goes wrong / status
    * restart e.g. line scan from current position?

* Tidy up app:
   * remove unused code

* move app to robot

* change direction of robot

* slow down near fence, speed up in the middle (may make crash prediction below unnecessary - if reducing heartbeat timeout in robot)
* keep going for large HDOP if far from fence
* use GPS hdop in tracking, figure out why HDOP is 0.5 when RTK
* looser cut-off on no RTK - either adjust error in GPS tracking (check out HDOP in RMC)
 - or - accept no GPS for a while - at least if heading is good and is not going to crash.
 estimate time to impact from last good RTK position?
BUT: also quicker than 5s heartbeat timeout, i.e. direct STOP command if about to crash and loose RTK 
* avoid going in circles on drifting GPS: HDOP -> tracker could solve that
* area filling - scan lines
* if HDOP high: alternative targets or wait for ok HDOP
* fence bump control: reverse out from fence

* automatic charging
* docking garage
* camera
* camera coverage of garden
* realsense - mapping, collision avoidance


Mission control dashboard
- map (draw AOI, speeds/behavior, move fence, path, density, ..., GPS/tracking error, heading error)
- system health: show currents, battery level
- (E)stop button
- Duration, Ah drawn (total, trip (since charge), button for reset trip charge), distance covered
- Mode: fence bump, lines, manual, manual override (temporarily)
- Manual controls
- Start mission - which mission
- app to monitor status, with notifications on issues

Analytics
- GPS/tracking precision
- Coverage
- Replay/show mission on map

Mapping
- find fence - show position on map with manual control - recorder mode....


Mission control
- run to area of "little" probability
- speed and cut speed as fn(x,y)
- avoid tail swinging out (reverse back out, put GPS above cutter and turn around cutter)
- estimate error based on odometry and GPS signal - keep on going if looses RTK
- TODO: check GPS error when stopped and no RTK


