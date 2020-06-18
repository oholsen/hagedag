# hagedag
DIY lawn mower robot



## TODO

* Max speed on motors: Too high speed+omega cannot do turn radius
* slow down near fence
* odometer only when no GPS fix -> higher uncertainty

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
   * refactor simulator to use tracker? If want to simulate HDOP issues...


* move app to robot

* change direction of robot

* slow down near fence, speed up in the middle (may make crash prediction below unnecessary - if reducing heartbeat timeout in robot)
* keep going for large HDOP if far from fence
* use GPS hdop in tracking
* looser cut-off on no RTK - either adjust error in GPS tracking (check out HDOP in RMC)
 - or - accept no GPS for a while - at least if heading is good and is not going to crash.
 estimate time to impact from last good RTK position?
BUT: also quicker than 5s heartbeat timeout, i.e. direct STOP command if about to crash and loose RTK 
* avoid going in circles on drifting GPS: HDOP -> tracker could solve that
* how to use hdop in line following - same applies: ok if near fence


* fence following control: make sure don't hit obstacles (obstacles are holes in the grass): grass shape has fence and holes
* area filling - scan lines or just fence following with restricting fence by path travelled ("cells filled")
  - handle multiple areas to fill - only use areas with area above threshold, e.g. 0.5 m2
  - handle travel/path between areas, path planning, not crashing into sandkasse, gran, hjørner, ...
* if HDOP high: alternative targets or wait for ok HDOP
* fence bump control: reverse out from fence
* revs sensor on cutter
* stuck detection: both on cutter and robot
* stop cutter outside fence?

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


