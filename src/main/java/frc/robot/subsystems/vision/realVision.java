package frc.robot.subsystems.vision;

import java.util.Optional;


import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.SystemManager;
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
    LimelightPoseEstimator frontVisionEstimate;
    LimelightPoseEstimator backVisionEstimate;
    StructPublisher<Pose3d> frontPosePublisher;
    StructPublisher<Pose3d> backPosePublisher;
    Optional<PoseEstimate> estimatedFrontPose;
    Optional<PoseEstimate> estimatedBackPose;
  


    /**
     * Creates limelights using the YALL library.
     *
     * Will be used to get vision estimations of where the robot is on the field when not in simulation
     *
     */
    public realVision() {

        limelightFront = new Limelight(Constants.limelightConstants.frontCameraName);
        limelightBack = new Limelight(Constants.limelightConstants.backCameraName);

        limelightFront.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Constants.limelightConstants.frontCameraPose)
         .save();
        
        limelightBack.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Constants.limelightConstants.backCameraPose)
         .save();


        frontVisionEstimate = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
        backVisionEstimate = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        frontPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("FrontPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));
        backPosePublisher = inst.getTable("SmartDashboard/Vision").getStructTopic("BackPoseEstimation", Pose3d.struct).publish(PubSubOption.keepDuplicates(true));

    }

    @Override
    public void periodic() {
        // These are called in periodic because they need to be according to YALL docs
        limelightBack.getSettings()
            .withRobotOrientation(
                new Orientation3d(
                    SystemManager.swerve.getRotation3d(),
                    new AngularVelocity3d(
                        DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                        DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                        DegreesPerSecond.of(SystemManager.swerve.getYawVelocity().baseUnitMagnitude()) // Yaw, rotation along the ground floor
                        )  
                )
            )
            .save();

        limelightFront.getSettings()
         .withRobotOrientation(
            new Orientation3d(
                SystemManager.swerve.getRotation3d(),
                new AngularVelocity3d(
                    DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
                    DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
                    DegreesPerSecond.of(SystemManager.swerve.getYawVelocity().baseUnitMagnitude()) // Yaw, rotation along the ground floor
                    )
            )
         )
         .save();


    }
    
    /**
     * Gets the front limelight's estimated position. Returns null if there is no position or if there is no connection
     *
     * @return {@link Pose3d} of where the front limelight thinks the robot is
     * 
     */
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

    /**
     * Gets the timestamp of the last found position of the front Limelight.
     *
     * @return {@link Double} of the timestamp of the last found position of the Limelight. Returns -1.0 if it cannot find a timestamp.
     */
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

    /**
     * Gets the back Limelight's estimated position. Returns null if there is no position or if there is no connection
     *
     * @return {@link Pose3d} of where the front Limelight thinks the robot is
     * 
     */
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

    /**
     * Gets the timestamp of the last found position of the back Limelight.
     *
     * @return {@link Double} of the timestamp of the last found position of the Limelight. Returns -1.0 if it cannot find a timestamp.
     */
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
