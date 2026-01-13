package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;

public class AimAtPoint extends Command {
    public Pose2d toAimAt;
    public AimAtPoint(Pose2d toAimAt){
        this.toAimAt = toAimAt;
        addRequirements(SystemManager.swerve);
    }

    @Override
    public void periodic(){
        Pose2d robotPose = 
        Rotation2d goal = Rotation2d.from
        SystemManager.swerve.drive(0, new Rotation2d(), gaol);
    }
}
