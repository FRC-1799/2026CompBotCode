package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GeneralManager;

public class stateBuddyCommand extends Command{

    GeneralManager.generalState buddyState;
    public stateBuddyCommand(GeneralManager.generalState buddyState){
        this.buddyState = buddyState;
    }

    @Override
    public void initialize(){
        GeneralManager.startState(buddyState);
    }

    public boolean isFinished(){
        return GeneralManager.getState()!=buddyState;
    }

    @Override
    public void end(boolean wasInterrupted){
        GeneralManager.cancelSpecificState(buddyState);
    }


}
