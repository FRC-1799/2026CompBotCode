// package frc.robot.subsystems.vision;

// import limelight.Limelight;
// import limelight.networktables.AngularVelocity3d;
// import limelight.networktables.LimelightPoseEstimator;
// import limelight.networktables.LimelightPoseEstimator.EstimationMode;
// import limelight.networktables.LimelightSettings.LEDMode;
// import limelight.networktables.Orientation3d;
// import limelight.networktables.PoseEstimate;

// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import java.util.Optional;

// import org.photonvision.estimation.VisionEstimation;

// import com.ctre.phoenix6.hardware.core.CorePigeon2;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// public class RealLimelight implements aprilTagInterface {
//     // Limelight limelightFront;
//     // Limelight limelightBack;
//     // SwerveSubsystem swerve;
//     // LimelightPoseEstimator frontVisionEstimate;
//     // LimelightPoseEstimator backVisionEstimate;
//     // StructPublisher<Pose3d> posePublisher;

//     // double robotYaw;
//     // SwerveSubsystem swerve;


//     public RealLimelight(SwerveSubsystem swerve) {
//         // limelightFront = new Limelight("limelightFront");
//         // limelightBack = new Limelight("limelightBack");

//         // limelightFront.getSettings()
//         //  .withLimelightLEDMode(LEDMode.PipelineControl)
//         //  .withCameraOffset(Pose3d.kZero)
//         //  .save();
        
//         // limelightBack.getSettings()
//         //  .withLimelightLEDMode(LEDMode.PipelineControl)
//         //  .withCameraOffset(Pose3d.kZero)
//         //  .save();

//         // this.swerve = swerve;


//         // StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
//         //     .getStructTopic("Vision/Pose", Pose3d.struct).publish();


//         // frontVisionEstimate = limelightFront.createPoseEstimator(EstimationMode.MEGATAG2);
//         // backVisionEstimate = limelightBack.createPoseEstimator(EstimationMode.MEGATAG2);

//     }

//     @Override
//     public void periodic() {
//         // limelightFront.getSettings()
//         //         .withRobotOrientation(new Orientation3d(swerve.getRotation3d(),
//         //                                                 new AngularVelocity3d(DegreesPerSecond.of(0), // Roll, our robot will not be front flipping
//         //                                                                     DegreesPerSecond.of(0), // Pitch, our robot will not be rolling around in mud
//         //                                                                     DegreesPerSecond.of(swerve.getYawVelocity().baseUnitMagnitude())))) // Yaw, rotation along the ground floor
//         //         .save();
                


//     }

//     @Override
//     public Pose3d getFrontPose() {
//         // Optional<PoseEstimate> estimatedFrontPose = frontVisionEstimate.getPoseEstimate();
//         // if (estimatedFrontPose.isEmpty()) {
//         //     return new Pose3d(0, 0, 0, null);
//         // }

//         // posePublisher.set(estimatedFrontPose.get().pose);
//         // return estimatedFrontPose.get().pose;

//         // Optional<PoseEstimate> visionEstimation = visionEstimate.getPoseEstimate();


//     }

//     @Override
//     public Pose3d getBackPose() {
//         Optional<PoseEstimate> estimatedBackPose = backVisionEstimate.getPoseEstimate();
//         if (estimatedBackPose.isEmpty()) {
//             return new Pose3d(0, 0, 0, null);
//         }
//         return estimatedBackPose.get().pose;
//     }

//     public Double getFrontTimestamp() {
//         return 0.0;
//     }

//     public Double getBackTimestamp() {
//         return 0.0;
//     }
    
// }
