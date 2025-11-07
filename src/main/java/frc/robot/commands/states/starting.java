package frc.robot.commands.states;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SystemManager;
import frc.robot.Utils.warningManager;
import frc.robot.subsystems.generalManager;

public class starting extends Command{

    /**creates a command that will bring the robot back to its starting config */
    public starting(){
        addRequirements(generalManager.subsystems);
    }

    /**initializes the command */
    @Override
    public void initialize(){
        SystemManager.elevator.setSetpoint(Constants.elevatorConstants.intakePosit);
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

    /**returns true once the bot has reached the starting config */
    @Override 
    public boolean isFinished(){
        return SystemManager.elevator.isAtSetpoint();
    }
}
