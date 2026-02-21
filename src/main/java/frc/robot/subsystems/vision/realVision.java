package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArraySubscriber;
import edu.wpi.first.networktables.BooleanArrayTopic;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import static edu.wpi.first.units.Units.DegreesPerSecond;


public class realVision implements aprilTagInterface{
    
    // public BooleanArraySubscriber[] coralLevelSubscribers;
    // public BooleanArraySubscriber[] algaeLevelSubscribers;
    // public BooleanArrayPublisher[] algaeLevelPublishers;
    // public final StructSubscriber<Pose3d> robotFrontPoseSubscriber;
    // public final StructSubscriber<Pose3d> robotBackPoseSubscriber;
    // public final DoubleSubscriber robotFrontTimestampSubscriber;
    // public final DoubleSubscriber robotBackTimestampSubscriber; 
    // DoubleTopic robotFrontTimestampTopic;
    // DoubleTopic robotBackTimestampTopic;
    Limelight limelightFront;
    Limelight limelightBack;
    SwerveSubsystem swerve;
    LimelightPoseEstimator frontVisionEstimate;
    LimelightPoseEstimator backVisionEstimate;
    StructPublisher<Pose3d> frontPosePublisher;
    StructPublisher<Pose3d> backPosePublisher;
    Optional<PoseEstimate> estimatedFrontPose;
    Optional<PoseEstimate> estimatedBackPose;
  



    public realVision(SwerveSubsystem swerve) {

        limelightFront = new Limelight("Limelight1");
        limelightBack = new Limelight("Limelight2");

        limelightFront.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .save();
        
        limelightBack.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .save();

        this.swerve = swerve;


        frontVisionEstimate = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
        backVisionEstimate = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        frontPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("FrontPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));
        backPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("BackPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));


    //     // Default values just in case no values are grabbed
    //     boolean[] reefDefaultList = {false, false, false, false, false, false, false, false, false, false, false, false};
    //     boolean[] algaeDefaultList = {false, false, false, false, false, false};
    //     double timestampDefault = 0.0;
    //     Pose3d robotDefaultPose = new Pose3d();

    //     NetworkTableInstance inst = NetworkTableInstance.getDefault();
    //     NetworkTable visionTable = inst.getTable("Vision");
    //     NetworkTable coralPositionTable = visionTable.getSubTable("CoralPositions");

    //     coralLevelSubscribers = new BooleanArraySubscriber[]{
    //         coralPositionTable.getBooleanArrayTopic("CoralL1").subscribe(reefDefaultList),
    //         coralPositionTable.getBooleanArrayTopic("CoralL2").subscribe(reefDefaultList),
    //         coralPositionTable.getBooleanArrayTopic("CoralL3").subscribe(reefDefaultList),
    //         coralPositionTable.getBooleanArrayTopic("CoralL4").subscribe(reefDefaultList)
    //     };
    //     algaeLevelSubscribers = new BooleanArraySubscriber[]{
    //             inst.getBooleanArrayTopic("GUI/AlgaeL1").subscribe(algaeDefaultList),
    //             inst.getBooleanArrayTopic("GUI/AlgaeL2").subscribe(algaeDefaultList)
    //     };

    //     algaeLevelPublishers = new BooleanArrayPublisher[]{
    //         inst.getBooleanArrayTopic("GUI/AlgaeL1").publish(),
    //         inst.getBooleanArrayTopic("GUI/AlgaeL2").publish( )
    // };



    //     // Gets the robot's position's subscriber
    //     StructTopic<Pose3d> robotFrontPoseTopic = visionTable.getStructTopic("FrontRobotPose", Pose3d.struct);
    //     robotFrontPoseSubscriber = robotFrontPoseTopic.subscribe(robotDefaultPose, PubSubOption.keepDuplicates(true));
    //     StructTopic<Pose3d> robotBackPoseTopic = visionTable.getStructTopic("BackRobotPose", Pose3d.struct);
    //     robotBackPoseSubscriber = robotBackPoseTopic.subscribe(robotDefaultPose, PubSubOption.keepDuplicates(true));

    //     // Gets the subscriber for the timestamp each position was published at
    //     robotFrontTimestampTopic = NetworkTableInstance.getDefault().getDoubleTopic("RobotPoseTimestampFront");
    //     robotFrontTimestampSubscriber = robotFrontTimestampTopic.subscribe(timestampDefault);
    //     robotBackTimestampTopic = NetworkTableInstance.getDefault().getDoubleTopic("RobotPoseTimestampBack");
    //     robotBackTimestampSubscriber = robotBackTimestampTopic.subscribe(timestampDefault);


    }

    @Override
    public void periodic() {
        limelightBack.getSettings()
        .withRobotOrientation(new Orientation3d(swerve.getRotation3d(),
                                                new AngularVelocity3d(DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                                                                    DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                                                                    DegreesPerSecond.of(swerve.getYawVelocity().baseUnitMagnitude())))) // Yaw, rotation along the ground floor
        .save();

        limelightFront.getSettings()
        .withRobotOrientation(new Orientation3d(swerve.getRotation3d(),
                                                new AngularVelocity3d(DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                                                                    DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                                                                    DegreesPerSecond.of(swerve.getYawVelocity().baseUnitMagnitude())))) // Yaw, rotation along the ground floor
        .save();


    }
    
    @Override
    public Pose3d getFrontPose() {
        estimatedFrontPose = frontVisionEstimate.getPoseEstimate();
        if (estimatedFrontPose.isEmpty()) {
            SmartDashboard.putBoolean("Vision/FrontPoseReadCorrectly", false);
            frontPosePublisher.set(Pose3d.kZero);
            return new Pose3d(0, 0, 0, Rotation3d.kZero);
            
        }
        SmartDashboard.putBoolean("Vision/FrontPoseReadCorrectly", true);
        

        frontPosePublisher.set(estimatedFrontPose.get().pose);
        return estimatedFrontPose.get().pose;
        // Pose3d robotFrontPose = robotFrontPoseSubscriber.get();
        // if (robotFrontPose.equals(new Pose3d())){
        //     return null;
        // }
        // SmartDashboard.putBoolean("poseReadCorrectly", false);

        // return robotFrontPose;
    }

    @Override
    public Double getFrontTimestamp() {
        Double timestamp;
        if (!estimatedFrontPose.isEmpty()) {
            timestamp = estimatedFrontPose.get().timestampSeconds;
            SmartDashboard.putNumber("Vision/received front timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/frontTimestampConnected", true);
        }
        else {
            timestamp = 0.0;
            SmartDashboard.putNumber("Vision/received front timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/frontTimestampConnected", false);
        }


        return timestamp;
    }

    @Override
    public Pose3d getBackPose() {
        estimatedBackPose = backVisionEstimate.getPoseEstimate();
        if (estimatedBackPose.isEmpty()) {
            SmartDashboard.putBoolean("Vision/BackPoseReadCorrectly", false);
            backPosePublisher.set(Pose3d.kZero);
            return new Pose3d(0, 0, 0, Rotation3d.kZero);
        }
        SmartDashboard.putBoolean("Vision/BackPoseReadCorrectly", true);
        

        backPosePublisher.set(estimatedBackPose.get().pose);
        return estimatedBackPose.get().pose;
    }

    @Override
    public Double getBackTimestamp() {
        Double timestamp;
        if (!estimatedBackPose.isEmpty()) {
            timestamp = estimatedBackPose.get().timestampSeconds;
            SmartDashboard.putNumber("Vision/received back timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/backTimestampConnected", true);
        }
        else {
            timestamp = 0.0;
            SmartDashboard.putNumber("Vision/received back timestamp", timestamp);
            SmartDashboard.putBoolean("Vision/backTimestampConnected", false);
        }


        return timestamp;
    }



}
