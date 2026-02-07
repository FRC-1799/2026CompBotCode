package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Limelight extends SubsystemBase{
    double robotYaw;
    SwerveSubsystem swerve;


    public Limelight(SwerveSubsystem swerve) {

        // Camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
            Constants.limelightConstants.forwardOffset.baseUnitMagnitude(), 
            Constants.limelightConstants.sideOffset.baseUnitMagnitude(), 
            Constants.limelightConstants.heightOffset.baseUnitMagnitude(),   
            Constants.limelightConstants.roll.baseUnitMagnitude(), 
            Constants.limelightConstants.pitch.baseUnitMagnitude(),
            Constants.limelightConstants.yaw.baseUnitMagnitude()  
        );
        this.swerve = swerve;
        addRequirements(swerve);

    }

    @Override
    public void periodic() {

        robotYaw = LimelightHelpers.SetRobotOrientation("", )
    }
    
}
