package frc.robot.subsystems.vision.calibration;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.calibration.spec.Limelight3A;
import frc.robot.subsystems.vision.calibration.spec.RobotSpec;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;

public class calibrationCommand extends Command{
    CameraCal llCal; 
    
    public calibrationCommand() {
        var cameras = Arrays.asList(
            new Camera("limelight-one", new Limelight3A()),
            new Camera("limelight-two", new Limelight3A())
        );

        var tp = new Pose3d(
            new Translation3d(0.86, 0.16, -0.10)  ,
            new Rotation3d(
                    Degrees.of(0),
                    Degrees.of(13),
                    Degrees.of(-10)
            )
        );

        llCal = new CameraCal(
                new RobotSpec(Meter.of(1), Meter.of(1), Meter.of(0.5)),
                cameras,
                Arrays.asList(new FiduciaryTag(35, tp))
        );




        // zero cameras
        llCal.SetDefaultCameraPositions();

        llCal.RobotPose = new Pose3d(0,0,0, new Rotation3d(0,0,0));

        
    }

    public void execute() {
        llCal.PublishData();
    }

    public void end() {}
}
