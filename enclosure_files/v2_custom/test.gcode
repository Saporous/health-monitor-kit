M190 S110 ; set bed temperature
M104 S230 ; set temperature
G28 ; home all axes
G92 E0 ; set all axes to 0
M109 S230 ; wait for temperature to be reached
G21 ; set units to millimeters
G90 ; use absolute coordinates
M82 ; use absolute distances for extrusion
G1 Z2.000 F100 ; lift nozzle slowly