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
import frc.robot.Constants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;


import static edu.wpi.first.units.Units.DegreesPerSecond;


public class realVision implements aprilTagInterface{
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

        limelightFront = new Limelight("limelight-one");
        limelightBack = new Limelight("limelight-two");

        limelightFront.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Constants.limelightConstants.frontCameraPose)
         .save();
        
        limelightBack.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Constants.limelightConstants.backCameraPose)
         .save();

        this.swerve = swerve;


        frontVisionEstimate = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
        backVisionEstimate = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        frontPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("FrontPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));
        backPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("BackPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));

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
            frontPosePublisher.set(null);
            return null;
            
        }
        SmartDashboard.putBoolean("Vision/FrontPoseReadCorrectly", true);
        

        frontPosePublisher.set(estimatedFrontPose.get().pose);
        return estimatedFrontPose.get().pose;
    }

    @Override
    public Double getFrontTimestamp() {
        Double timestamp;
        if (!estimatedFrontPose.isEmpty()) {
            timestamp = estimatedFrontPose.get().timestampSeconds;
            SmartDashboard.putNumber("Vision/ReceivedFrontTimestamp", timestamp);
            SmartDashboard.putBoolean("Vision/FrontTimestampConnected", true);
        }
        else {
            timestamp = -1.0;
            SmartDashboard.putNumber("Vision/ReceivedFrontTimestamp", timestamp);
            SmartDashboard.putBoolean("Vision/FrontTimestampConnected", false);
        }


        return timestamp;
    }

    @Override
    public Pose3d getBackPose() {
        estimatedBackPose = backVisionEstimate.getPoseEstimate();

        if (estimatedBackPose.isEmpty()) {
            SmartDashboard.putBoolean("Vision/BackPoseReadCorrectly", false);
            backPosePublisher.set(null);
            return null;
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
            SmartDashboard.putNumber("Vision/ReceivedBackTimestamp", timestamp);
            SmartDashboard.putBoolean("Vision/BackTimestampConnected", true);
        }
        else {
            timestamp = -1.0;
            SmartDashboard.putNumber("Vision/ReceivedBackTimestamp", timestamp);
            SmartDashboard.putBoolean("Vision/BackTimestampConnected", false);
        }


        return timestamp;
    }



}
