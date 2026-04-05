package frc.robot.commands.AutoStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SystemManager;

public class SemiAutoState extends Command{
    public enum semiAutoStateState{
        inFirstCommand,
        handoff,
        inSecondCommand
    }

    Command firstCommand;
    Command secondCommand;

    Command currentCommand = firstCommand;
    BooleanSupplier canHandoff;
    boolean cancelHandoff;
    semiAutoStateState state = semiAutoStateState.inFirstCommand;

    boolean canFinishFlag = true;

    public SemiAutoState (Command driveCommand){
        this(driveCommand, new InstantCommand(), ()->true);
    }

    public SemiAutoState(Command driveCommand, Command handoffCommand){
        this(driveCommand, handoffCommand, ()->true);
    }

    public SemiAutoState(Command driveCommand, Command handoffCommand, BooleanSupplier canHandoff){
        this(driveCommand, handoffCommand, canHandoff, true);
    }

    public SemiAutoState(Command driveCommand, Command handoffCommand, BooleanSupplier canHandoff, boolean cancelHandoff){
        this.firstCommand=driveCommand;
        this.secondCommand=handoffCommand;
        this.canHandoff=canHandoff;
        this.cancelHandoff=cancelHandoff;
        currentCommand = firstCommand;
        addRequirements(firstCommand.getRequirements());
        addRequirements(secondCommand.getRequirements());

    }

    @Override
    public void initialize(){
        state = semiAutoStateState.inFirstCommand;
        currentCommand = firstCommand;
        currentCommand.initialize();
    }

    @Override
    public void execute(){
        canFinishFlag=true;

        if (currentCommand.isFinished()){
            if (state==semiAutoStateState.inFirstCommand){
                if (!canHandoff.getAsBoolean()){
                    state = semiAutoStateState.handoff;
                    currentCommand = new WaitUntilCommand(canHandoff);
                }

                else{
                    state = semiAutoStateState.inSecondCommand;
                    currentCommand = secondCommand;
                    currentCommand.initialize();
                }
                SystemManager.swerve.lock();
            }

            else if (state == semiAutoStateState.handoff){
                currentCommand = secondCommand;
                currentCommand.initialize();
            }
            
        }



        if (!currentCommand.isFinished()){
            currentCommand.execute();
        }


        
    }

    @Override
    public boolean isFinished(){
        return state==semiAutoStateState.inSecondCommand && secondCommand.isFinished() && canFinishFlag;
    }

    @Override
    public void end(boolean wasInterrupted){
        currentCommand.end(wasInterrupted);
    }

}
