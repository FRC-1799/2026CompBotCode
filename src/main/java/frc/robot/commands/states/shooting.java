package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class shooting extends Command{
    public Pose2d goal; 

    boolean isShooting;
    Command driveCommand;

    public shooting(){
        addRequirements(GeneralManager.subsystems);
    }

    @Override 
    public void initialize(){
        Pose2d swervePose = SystemManager.getSwervePose();
        if (
            swervePose.getX()>FieldPosits.bottomAllianceDSCorner.getX()&& swervePose.getY() > FieldPosits.bottomAllianceDSCorner.getY() &&
            swervePose.getX()<FieldPosits.topAllianceMidCorner.getX()&& swervePose.getY()<FieldPosits.topAllianceMidCorner.getY()
        ){
            goal = new Pose2d(swervePose.getTranslation(), utilFunctions.getAngleBetweenTwoPoints(swervePose, FieldPosits.hubPose2d));
        }
        else{
            Translation2d goalTrans = swervePose.nearest(FieldPosits.scoringPoses).getTranslation();
            goal = new Pose2d(goalTrans, utilFunctions.getAngleBetweenTwoPoints(new Pose2d(goalTrans, new Rotation2d()), FieldPosits.hubPose2d));
        }

        driveCommand = SystemManager.swerve.driveToPose(goal);
        CommandScheduler.getInstance().schedule(driveCommand);

        SystemManager.shooter.setGoal(FieldPosits.hubPose3d);
        isShooting=false;

    }
    @Override
    public void execute(){
        Pose2d error = SystemManager.getSwervePose().relativeTo(goal);

        
        if (utilFunctions.pythagorean(error.getX(), error.getY())<AutonConstants.autoDriveScoreTolerance && error.getRotation().getDegrees()<AutonConstants.angleTolerance){
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
