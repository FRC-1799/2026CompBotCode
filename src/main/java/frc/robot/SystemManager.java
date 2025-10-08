package frc.robot;

import static edu.wpi.first.units.Units.Radians;

import java.io.File;
import org.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.autoManager;
import frc.robot.subsystems.generalManager;
import frc.robot.subsystems.AlgaeRemover.algaeRemoverInterface;
import frc.robot.subsystems.AlgaeRemover.realAlgaeRemover;
import frc.robot.subsystems.AlgaeRemover.simAlgaeRemover;
import frc.robot.subsystems.CoralGUI.compassGUI;
import frc.robot.subsystems.CoralGUI.coralGUI;
import frc.robot.subsystems.blinkin.blinkinInterface;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.intake.intakeIO;
import frc.robot.subsystems.intake.realIntake;
import frc.robot.subsystems.intake.simIntake;
import frc.robot.subsystems.lidar.lidarInterface;
import frc.robot.subsystems.swervedrive.AIRobotInSimulation;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.realSimulatedDriveTrain;
import frc.robot.subsystems.vision.aprilTagInterface;
import frc.robot.subsystems.vision.photonSim;
import frc.robot.subsystems.vision.realVision;
import frc.robot.subsystems.vision.reefIndexerIO;
import frc.robot.subsystems.vision.simReefIndexer;


public class SystemManager{
    public static SwerveSubsystem swerve;

    public static Field2d field;
    public static SimulatedArena simField;
    public static AIRobotInSimulation fakeBot;
    public static boolean hasNote = false;
    public static intakeIO intake;
    public static aprilTagInterface aprilTag;
    public static elevator elevator;
    public static reefIndexerIO reefIndexer;
    public static lidarInterface lidar;
    public static realSimulatedDriveTrain simButRealTrain = null;
    public static realVision realVisTemp = null;
    public static blinkinInterface blinkin;
    public static compassGUI compass;
    public static Robot robot;

    public static algaeRemoverInterface algaeRemover;

    
    // Add a Coral Array object for tracking
    public static coralGUI coralArray;

    /** Initializes the system manager along with all the systems on the robot */
    public static void SystemManagerInit(Robot robotIn){
        robot=robotIn;
        SimulatedArena.ALLOW_CREATION_ON_REAL_ROBOT = Constants.simConfigs.robotCanBeSimOnReal;

        // creates the swerve drive. Due to the complexity of the swerve system, it handles simulation differently and does not need an if-else block
        swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),  "swerve"));
        swerve.resetOdometry(Constants.driveConstants.startingPosit);
        
        field = new Field2d();
        SmartDashboard.putData("Field", field);


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

        elevator = new elevator();
        



        // Reef indexer
        // if (Constants.simConfigs.reefIndexerShouldBeSim){
        //     reefIndexer = new simReefIndexer();
        // } else {
        //     if (realVisTemp != null){
        //         reefIndexer = realVisTemp;
        //     } else {
        //     }
        // }
        reefIndexer = new realVision();


        // //Lidar
        // if (Constants.simConfigs.lidarShouldBeSim){
        //     lidar = new simLidar();
        // } else {
        //     // Lidar initialization if not sim
        // }

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
        

        if (Constants.simConfigs.algaeRemoverShouldBeSim){
            algaeRemover= new simAlgaeRemover();
        }
        else{
            algaeRemover = new realAlgaeRemover();
        }

        // Intake
        if (Constants.simConfigs.intakeShouldBeSim){
            if (RobotBase.isReal()){
                simButRealTrain = new realSimulatedDriveTrain();
            }
            intake = new simIntake();
        } else {
            intake = new realIntake();
        }
        

        //initializes and distributes the managers

        generalManager.generalManagerInit();
        autoManager.autoManagerInit();
        

        // Initialize Coral Array
        coralArray = new coralGUI();
        compass = new compassGUI();
    }

    /** Calls periodic on all the systems that do not inherit subsystem base. This function should be called in robot periodic */
    public static void periodic(){
        generalManager.periodic();
        autoManager.periodic();
        reefIndexer.periodic();
        coralArray.periodic();
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

    /** Returns the pose3 of a coral in the intake */
    public static Pose3d getIntakePosit(){
        return new Pose3d(getSwervePose()).plus(new Transform3d(intake.getTranslation(), new Rotation3d(0, Constants.elevatorConstants.angle.in(Radians) + Math.PI / 2, Math.PI)));
    }
}
