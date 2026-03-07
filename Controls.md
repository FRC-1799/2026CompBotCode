# WHILE THIS DOCUMENT SHOULD BE UP TO DATE PLEASE DO NOT UNCONDITIONALLY TRUST IT. YOU SHOULD STILL NOT BE INPUTTING ANYTHING WHILE CLOSE TO THE ROBOT UNLESS YOU CAN SEE **IN CODE** THAT IT IS SAFE.  


## Outlines the various control schemes.
Control schemes can be selected via the sendable chooser published to network tables at "/SmartDashboard/Control chooser"


## Default
This control scheme does literally nothing (auto will still run in autonomous mode). This ensures that drivers must actively choose and understand what control scheme they are using instead of making assumptions that may not be correct. This helps prevent dangerous missinputs. 

## Test Control (This control scheme changes often and so may not always be up to date)
Field oriented drive with left stick for directional control and right stick for rotation
Left Trigger: Smart shoot (Will only fire if the fuel is predicted to land within an active period)
Right Trigger: Intake
A: Begins passing
B: Outtakes game pieces through the intake 

## Rock Control
Robot oriented drive with left stick for directional control and right stick x axis for rotation
Left Trigger above 10%: AutoDrive and aimbot 
Left Trigger above 50% : Smart shoot (Will only fire if the fuel is predicted to land within an active period)(If the robot is not in a correct position to shoot this input will auto aim before firing)
Right Trigger: begins intaking
A: Starts passing

