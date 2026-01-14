package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.Utils.utilFunctions;

public class AimAtPoint extends Command {
    public Pose2d toAimAt;
    public StructPublisher<Rotation2d> rotationGoalPublisher = NetworkTableInstance.getDefault().getStructTopic("swerveDrive/RotationGoal", Rotation2d.struct).publish();

    public AimAtPoint(Pose2d toAimAt){
        this.toAimAt = toAimAt;
        addRequirements(SystemManager.swerve);
    }

    @Override
    public void execute(){
        rotationGoalPublisher.set(utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), toAimAt));
        SystemManager.swerve.drive(0, new Rotation2d(), utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), toAimAt));
    }
}
