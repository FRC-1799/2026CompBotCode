package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.simConfigs;
import frc.robot.Constants.AutonConstants;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;
import frc.robot.subsystems.TimingManager;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.realIntake;
import frc.robot.subsystems.Intake.simIntake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.realShooter;
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
    public static Intake intake;
    public static Shooter shooter;
    public static Robot robot;

    public static TimingManager clock = TimingManager.getInstance();

    public static Pose2d autoDriveGoal=new Pose2d();
    public static StructPublisher<Pose2d> autoDriveGoalPublisher = NetworkTableInstance.getDefault().getStructTopic("SmartDashboard/AutoDrive/goal", Pose2d.struct).publish();




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
            aprilTag = new realVision();
            //((realVision)aprilTag).initLimelightForwarding();
        }


        // Create an imaginary robot
        if (!RobotBase.isReal()){
            AIRobotInSimulation.startOpponentRobotSimulations();
            fakeBot = AIRobotInSimulation.getRobotAtIndex(0);
            // Overrides the default simulation
        }
        else{
            simButRealTrain = new realSimulatedDriveTrain();
        }



        if (simConfigs.intakeShouldBeSim){
            intake = new simIntake();
        }
        else{
            intake = new realIntake();
        }

       
        if (simConfigs.shooterShouldBeSim){
            shooter = new simShooter();
        }
        else {
            shooter = new realShooter();
        }


        //initializes and distributes the managers

        GeneralManager.generalManagerInit();
        


    }

    /** Calls periodic on all the systems that do not inherit subsystem base. This function should be called in robot periodic */
    public static void periodic(){
        SmartDashboard.putNumber("Score", score);
        SmartDashboard.putBoolean("AutoDrive/atGoal", swerveIsAtGoal());

        autoDriveGoalPublisher.set(autoDriveGoal);
        GeneralManager.periodic();
        TimingManager.getInstance().periodic();
        aprilTag.periodic();
        
    }

    /** @return the current pose of the robot */
    public static Pose2d getSwervePose(){
        return swerve.getPose();
    }

    // public static void initialPortForward() {
    //     if (Constants.simConfigs.aprilTagShouldBeSim) {
    //         ((realVision)aprilTag) .limelightForwarding(Constants.limelightConstants.limelight1Name);
    //     } 
    // }

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

    public static boolean swerveIsAtGoal(){
        Pose2d error = SystemManager.getSwervePose().relativeTo(autoDriveGoal);
        //System.out.println(error);
        return utilFunctions.pythagorean(error.getX(), error.getY())<AutonConstants.autoDriveScoreTolerance &&
             Math.abs(autoDriveGoal.getRotation().getDegrees()-SystemManager.getSwervePose().getRotation().getDegrees())<AutonConstants.angleTolerance;
    }


}
