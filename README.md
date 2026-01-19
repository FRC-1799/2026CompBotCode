The robot code for 1799 in the first robotics 2026 game Rebuilt.

For more information on configuring the simulation see Sim setup.md.


To Get the robot running in simulation
1)Download WIPlIB VSCode at https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html  
2)Clone the 2026 github repo from https://github.com/FRC-1799/2026CompBotCode  
3)Open the GitHub repo in WPILIB VSCode  
4)Swap branches to localization  
5)Click the three dots in the top right and select simulate robot code(you must be on wifi to do this)  
6)After the code builds, a prompt should pop up in VSCode, select open in Sim GUI  
7)After a few seconds, a new tab should open  
8)Open advantage scope(one of the wpilib tools)  
9)In Advantage Scope, select file and then connect to simulation   
10)The top bar of Advantage Scope should start moving once it has connected  
11)In the 3D field tab, search and Robot pose and drag it into the open space to the bottom right  
12)Search and add coralPoses and algaePoses in the same way robot poses were added  
13)Search elevator and drag it under robot pose   
14)Go back to the sim GUI, in the top right, there should be a list of options including teliop, disconnected, and autonomous. Select autonomous.  
15)If everything went well, the robot model in Advantage Scope should start moving and scoring.  
