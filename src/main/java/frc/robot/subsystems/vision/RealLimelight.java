package frc.robot.subsystems.vision;

import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import java.util.Optional;

import org.photonvision.estimation.VisionEstimation;

import com.ctre.phoenix6.hardware.core.CorePigeon2;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RealLimelight implements aprilTagInterface {
    Limelight limelight;
    SwerveSubsystem swerve;
    LimelightPoseEstimator visionEstimate;
    CorePigeon2 gyro;

    // double robotYaw;
    // SwerveSubsystem swerve;


    public RealLimelight(SwerveSubsystem swerve) {
        limelight = new Limelight("limelight");

        limelight.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .save();

        this.swerve = swerve;

        gyro = (CorePigeon2) swerve.getIMU();





        visionEstimate = limelight.createPoseEstimator(EstimationMode.MEGATAG1);

        // // Camera pose relative to robot center (x forward, y left, z up, degrees)
        // LimelightHelpers.setCameraPose_RobotSpace("",
        //     Constants.limelightConstants.forwardOffset.baseUnitMagnitude(), 
        //     Constants.limelightConstants.sideOffset.baseUnitMagnitude(), 
        //     Constants.limelightConstants.heightOffset.baseUnitMagnitude(),   
        //     Constants.limelightConstants.roll.baseUnitMagnitude(), 
        //     Constants.limelightConstants.pitch.baseUnitMagnitude(),
        //     Constants.limelightConstants.yaw.baseUnitMagnitude()  
        // );
        // this.swerve = swerve;
        // addRequirements(swerve);

    }

    @Override
    public void periodic() {
        limelight.getSettings()
                .withRobotOrientation(new Orientation3d(gyro.get(),
                                                        new AngularVelocity3d(DegreesPerSecond.of(gyro.getPitchVelocity()),
                                                                            DegreesPerSecond.of(gyro.getRollVelocity()),
                                                                            DegreesPerSecond.of(gyro.getYawVelocity()))))
                .save();
                


    }

    @Override
    public Pose3d getFrontPose() {
        Optional<PoseEstimate> visionEstimation = visionEstimate.getPoseEstimate();
        if (visionEstimation.isEmpty()) {
            return new Pose3d(0, 0, 0, null);
        }
        return visionEstimation.get().pose;

    }

    @Override
    public Pose3d getBackPose() {
        Optional<PoseEstimate> visionEstimation = visionEstimate.getPoseEstimate();
        if (visionEstimation.isEmpty()) {
            return new Pose3d(0, 0, 0, null);
        }
        return visionEstimation.get().pose;
    }

    
}
