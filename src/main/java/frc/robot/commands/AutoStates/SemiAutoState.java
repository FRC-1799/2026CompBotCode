package frc.robot.commands.AutoStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SystemManager;

public class SemiAutoState extends Command{
    public enum semiAutoStateState{
        firstCommand,
        handoff,
        secondCommand
    }

    Command firstCommand;
    Command secondCommand;
    BooleanSupplier canHandoff;
    boolean cancelHandoff;
    semiAutoStateState state = semiAutoStateState.firstCommand;

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

    }

    @Override
    public void initialize(){
        state = semiAutoStateState.firstCommand;
        firstCommand.schedule();
    }

    @Override
    public void execute(){
        canFinishFlag=true;


        if (state == semiAutoStateState.firstCommand){
            if (SystemManager.swerveIsAtGoal()){
                state=semiAutoStateState.handoff;
                if (!canHandoff.getAsBoolean()){
                    new InstantCommand(()->SystemManager.swerve.lock()).until(canHandoff::getAsBoolean).schedule();

                }
            }
            if (!firstCommand.isScheduled()){
                firstCommand.schedule();
            }
        }

        if (state == semiAutoStateState.handoff){
            if (canHandoff.getAsBoolean()){
                state=semiAutoStateState.secondCommand;
                secondCommand.schedule();
                canFinishFlag=false;
            }
        }

        if (state == semiAutoStateState.secondCommand){
            if (cancelHandoff&&!canHandoff.getAsBoolean()){
                secondCommand.cancel();
                state = semiAutoStateState.handoff;
            }
        }


        
    }

    @Override
    public boolean isFinished(){
        return state==semiAutoStateState.secondCommand && !secondCommand.isScheduled() && canFinishFlag;
    }

    @Override
    public void end(boolean wasInterrupted){
        if (state==semiAutoStateState.firstCommand){
            firstCommand.cancel();
        }
        else if (state==semiAutoStateState.secondCommand){
            secondCommand.cancel();
        }
    }

}
