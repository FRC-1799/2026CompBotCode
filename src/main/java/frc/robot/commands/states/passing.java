package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.shooterConstants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class passing  extends Command{

    public Pose3d goal; 

    boolean isShooting;

    public passing(){
        addRequirements(GeneralManager.subsystems);
        addRequirements(SystemManager.swerve);
    }

    @Override 
    public void initialize(){
        isShooting=false;
        goal = new Pose3d(SystemManager.getSwervePose().nearest(FieldPosits.passingPoses));




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
    }

    @Override
    public void end(boolean wasCanceled){
        SystemManager.shooter.stop();
        GeneralManager.endCallback(wasCanceled);
    }
}
