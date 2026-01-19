## Downloads:
newest wpilib vscode
newest code version from https://github.com/FRC-1799/2026CompBotCode/releases/

## Running the Simulator:
Open the newest version of the robot code in WPIlib vscode. 
Click the three dots at the top right and then click "simulate robot code".
Wait for the code to compile and then click continue when prompted.

If this does not work make sure that:
    A) You are using WPIlib vscode and not normal VScode
    B) The code is open in the correct directory (VScode should show files like build.gradle and README.md in the root directory).

## Running Advantage Scope for visualization.
Open the version of advantage scope that came with WPILib VScode. Then import the advantage scope layout included with this code base under the name "DriverAdvantageScopeLayout.json". Finally connect to the simulator using the prompt in the file section of the top bar or by using ctrl+shift+K.

## Running Elastic for robot data.
Open the version of Elastic that came with WPILib VScode. Then import the layout include with this code base under the name "DriverElastic.json". Elastic should connect automatically but if not open settings and make sure that IP Address mode is set to Local Host. 

## Some common issues
Make sure that 
    A) Make sure the controller is selected in slot one of the sim gui. Otherwise the robot will not receive your inputs. 
    B) Make sure the correct control scheme has been selected. Otherwise the robot may not respond correctly. 
    C) Make sure that you currently have the most recent version of the code downloaded from github. 
    D) Make sure the robot is understanding your controls in the expected way. The general state value on Elastic will tell you what the robot is currently trying to do. If this does not match your expectations see Controls.md 
    

## Finally remember that google is your friend.
FRC has a ton of great online docs and it will get you an answer A LOT faster for simple problems. You are welcome to ask for help but please google your problems first.  