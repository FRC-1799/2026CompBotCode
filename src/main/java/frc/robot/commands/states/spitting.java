package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;
import frc.robot.subsystems.GeneralManager;

public class spitting extends Command{
    public spitting(){
        addRequirements(GeneralManager.subsystems);
    }

    @Override
    public void initialize(){
        SystemManager.shooter.startShooting();
    }

    @Override
    public void end(boolean wasCanceled){
        SystemManager.shooter.stop();
    }
}
