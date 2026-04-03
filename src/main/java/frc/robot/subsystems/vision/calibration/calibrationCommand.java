package frc.robot.subsystems.vision.calibration;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.calibration.spec.Limelight3A;
import frc.robot.subsystems.vision.calibration.spec.RobotSpec;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

public class calibrationCommand extends Command{
    CameraCal limelightCalibration; 
    
    public calibrationCommand() {
        List<Camera> cameras = Arrays.asList(
            new Camera("limelight-one", new Limelight3A()),
            new Camera("limelight-two", new Limelight3A())
        );

        Pose3d tagPose = new Pose3d(
            new Translation3d(2, 0, 0.57)  ,
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(0),
                    Degrees.of(0)
            )
        );

        limelightCalibration = new CameraCal(
                new RobotSpec(Meter.of(1), Meter.of(1), Meter.of(0.5)),
                cameras,
                Arrays.asList(new FiduciaryTag(35, tagPose))
        );




        // zero cameras
        limelightCalibration.SetDefaultCameraPositions();

        limelightCalibration.RobotPose = new Pose3d(0,0,0, new Rotation3d(0,0,0));

    }

    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Calibrate/CalIsRunning", true);

    }

    @Override
    public void execute() {
        limelightCalibration.update();
    }


    @Override
    public void end(boolean wasInterupted) {
        SmartDashboard.putBoolean("Calibrate/CalIsRunning", false);
    }
}
