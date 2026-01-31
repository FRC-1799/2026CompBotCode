package frc.robot.commands.states;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.GeneralManager;

public class intaking  extends Command{
    public intaking(){
        addRequirements(GeneralManager.subsystems);
    }

    @Override
    public void initialize(){
        SystemManager.intake.start();

    }

    @Override
    public void end(boolean wasInterrupted){
        SystemManager.intake.stop();
        GeneralManager.endCallback(wasInterrupted);

    }
}
