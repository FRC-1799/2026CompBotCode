package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;
import frc.robot.subsystems.generalManager;

public class resting extends Command{

    /**creates a command that does nothing but hold the space a state would so that other commands can not sneak into the mechanisms */
    public resting(){
        addRequirements(generalManager.subsystems);
    }

    @Override
    public void initialize(){
    }

    /**called ever rio cycle while the command is scheduled*/
    @Override
    public void execute(){

    }

    /**
     * command called when the command finishes
     * @param wasInterrupted wether or not the command was canceled
    */
    @Override
    public void end(boolean wasInterrupted){
        generalManager.endCallback(wasInterrupted);
    }
}
