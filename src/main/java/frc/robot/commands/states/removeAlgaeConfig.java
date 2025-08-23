

package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldPosits.reefLevel.algaeRemovalEnum;
import frc.robot.SystemManager;
import frc.robot.subsystems.generalManager;

public class removeAlgaeConfig extends Command {
    algaeRemovalEnum level;

    public removeAlgaeConfig(algaeRemovalEnum level){
        this.level=level;
        addRequirements(generalManager.subsystems);
    }

    /**initializes the command */
    @Override
    public void initialize(){
        SystemManager.elevator.setSetpoint(level.getElevatorValue());
        
    }


    /**@return true once the robot has entered the correct config */
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
