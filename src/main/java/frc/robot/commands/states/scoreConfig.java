package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SystemManager;
import frc.robot.FieldPosits.reefLevel;
import frc.robot.Utils.warningManager;
import frc.robot.subsystems.generalManager;


public class scoreConfig extends Command{
    protected reefLevel scoreLevel;

    /**
     * Creates a command to configure mechanisms to specified level. DOES NOT ACTUALLY OUTTAKE 
     * @param level the level to configure
     */
    public scoreConfig(reefLevel level){
        scoreLevel=level;
        addRequirements(generalManager.subsystems);
    }

    /**initializes the command */
    @Override
    public void initialize(){
        SystemManager.elevator.setSetpoint(scoreLevel.getElevatorValue());
        SystemManager.intake.stop();
    }


    /**called ever rio cycle while the command is scheduled*/
    @Override 
    public void execute(){
        if (generalManager.getStateCommand()!=this){
            warningManager.throwAlert(warningManager.generalRoutineCalledManually);
            cancel();
        }
    }

    /**returns true when the mec is configured to the requested level */
    @Override
    public boolean isFinished(){
        return SystemManager.elevator.isAtSetpoint();
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
