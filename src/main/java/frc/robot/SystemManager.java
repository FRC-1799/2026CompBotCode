package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.autoManager;
import frc.robot.Constants.simConfigs;
import frc.robot.subsystems.GeneralManager;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.realIntake;
import frc.robot.subsystems.Intake.simIntake;
import frc.robot.subsystems.Shooter.simShooter;
import frc.robot.subsystems.lidar.lidarInterface;
import frc.robot.subsystems.lidar.realLidar;
import frc.robot.subsystems.lidar.simLidar;
import frc.robot.subsystems.swervedrive.AIRobotInSimulation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.realSimulatedDriveTrain;
import frc.robot.subsystems.vision.aprilTagInterface;
import frc.robot.subsystems.vision.photonSim;
import frc.robot.subsystems.vision.realVision;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;


public class SystemManager{
    public static SwerveSubsystem swerve;

    public static SimulatedArena simField;
    public static AIRobotInSimulation fakeBot;
    public static aprilTagInterface aprilTag;

    public static realSimulatedDriveTrain simButRealTrain = null;
    public static realVision realVisTemp = null;
    public static Intake intake;
    public static simShooter shooter;
    public static Robot robot;

    protected static int score = 0;
    
    // Add a Coral Array object for tracking

    /** Initializes the system manager along with all the systems on the robot */
    public static void SystemManagerInit(Robot robotIn){
        robot=robotIn;
        SimulatedArena.ALLOW_CREATION_ON_REAL_ROBOT = Constants.simConfigs.robotCanBeSimOnReal;

        // creates the swerve drive. Due to the complexity of the swerve system, it handles simulation differently and does not need an if-else block
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),  "swerve"));
        swerve.resetOdometry(Constants.driveConstants.startingPosit);
        


        // Initializes all the systems
        // Each block should initialize one system as either real or imaginary based on the constants value 
    



        // April tags
        if (Constants.simConfigs.aprilTagShouldBeSim){
            aprilTag = new photonSim();
        } else {
            realVisTemp = new realVision();
            aprilTag = realVisTemp;
        }

        // Elevator

        



        
        


        // Blinkin
        // if(Constants.simConfigs.blinkinShouldBeSim){
        //     blinkin = new simBlinkin();
        // } else {
        //     blinkin = new realBlinkin();
        // }

        // Create an imaginary robot
        if (!RobotBase.isReal()){
            AIRobotInSimulation.startOpponentRobotSimulations();
            fakeBot = AIRobotInSimulation.getRobotAtIndex(0);
            // Overrides the default simulation
        }
        else{
            simButRealTrain = new realSimulatedDriveTrain();
        }
        if (simConfigs.intakeShouldBeSim) intake = new simIntake();
        else intake = new realIntake();

        shooter = new simShooter();



        //initializes and distributes the managers

        GeneralManager.generalManagerInit();
        autoManager.autoManagerInit();
        


    }

    /** Calls periodic on all the systems that do not inherit subsystem base. This function should be called in robot periodic */
    public static void periodic(){
        SmartDashboard.putNumber("Score", score);


        GeneralManager.periodic();
        autoManager.periodic();
    }

    /** @return the current pose of the robot */
    public static Pose2d getSwervePose(){
        return swerve.getPose();
    }

    /** @return the pose of the simulated maplesim drive. If the drivetrain is real, then the function will just return the pose estimator's pose */
    public static Pose2d getRealPoseMaple(){
        if (RobotBase.isReal()){
            return getSwervePose();
        }
        return swerve.getMapleSimPose();
    }

    public static void addScore(int toAdd){
        score+=toAdd;
    }

    public static void setScore(int toSet){
        score=toSet;
    }

    public static int getScore(){
        return score;
    }



}
