package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemManager;
import frc.robot.subsystems.generalManager;


public class intaking extends Command{
    public boolean isInIntakingPhase;

    /**
     * Command to manage the intaking state
     */
    public intaking(){
        addRequirements(generalManager.subsystems);
    }

    /**initializes the command */
    @Override
    public void initialize(){
        if (SystemManager.intake.hasPiece()){
            cancel();
        }
        isInIntakingPhase=false;

        SystemManager.elevator.setSetpoint(Constants.elevatorConstants.intakePosit);
    }


    /**called ever rio cycle while the command is scheduled*/
    @Override
    public void execute(){
        if (!isInIntakingPhase && SystemManager.elevator.isAtSetpoint() ){
            SystemManager.intake.intake();
            isInIntakingPhase=true;
        }    
    }

    /**@return true once the robot has acquired a piece */
    @Override
    public boolean isFinished(){
        return SystemManager.intake.hasPiece();
    }


    /**
     * command called when the command finishes
     * @param wasInterrupted wether or not the command was canceled
    */
    @Override
    public void end(boolean wasInterrupted){
        generalManager.endCallback(wasInterrupted);
        SystemManager.intake.stop();
    }
}
