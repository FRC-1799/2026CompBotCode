package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.shooterConstants;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class shooting extends Command{
    public Pose3d goal; 

    boolean isShooting;

    public shooting(){
        goal = FieldPosits.hubPose3d ;
        addRequirements(GeneralManager.subsystems);
        addRequirements(SystemManager.swerve);
    }

    @Override 
    public void initialize(){
        SystemManager.shooter.setGoal(goal);
        isShooting=false;

    }
    @Override
    public void execute(){
        SystemManager.swerve.drive(0, new Rotation2d(), utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), goal.toPose2d()));

        if (Math.abs(
                SystemManager.getSwervePose().getRotation().getDegrees() -
                utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), goal.toPose2d()).getDegrees())
            <shooterConstants.shootingTolerance.in(Degrees)){
            SystemManager.shooter.startShooting();
        }
        else{
            SystemManager.shooter.stop();
        }
        System.out.println("hiiiii");
    }

    @Override
    public void end(boolean wasCanceled){
        SystemManager.shooter.stop();
        GeneralManager.endCallback(wasCanceled);
    }
}
