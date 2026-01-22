package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.GeneralManager;

public class intaking  extends Command{
    protected Command driveCommand;
    public intaking(){
        addRequirements(GeneralManager.subsystems);
    }

    @Override
    public void initialize(){
        Pose2d swervePose = SystemManager.getSwervePose();
        SystemManager.simIntake.start();
        if (
            swervePose.getX()>FieldPosits.bottomAllianceDSCorner.getX()&& swervePose.getY() > FieldPosits.bottomAllianceDSCorner.getY() &&
            swervePose.getX()<FieldPosits.topAllianceMidCorner.getX()&& swervePose.getY()<FieldPosits.topAllianceMidCorner.getY()
        ){
            driveCommand =  SystemManager.swerve.driveToPose(swervePose.nearest(FieldPosits.intakingHandoffPoses), AutonConstants.intakeHandoffSpeed);
            driveCommand.schedule();
        }
    }

    @Override
    public void end(boolean wasInterrupted){
        SystemManager.simIntake.stop();
        GeneralManager.endCallback(wasInterrupted);
        if (driveCommand!=null){
            CommandScheduler.getInstance().cancel(driveCommand);
        }
    }
}
