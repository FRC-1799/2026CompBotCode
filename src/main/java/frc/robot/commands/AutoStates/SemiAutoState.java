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

    Command currentCommand = firstCommand;
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
        currentCommand = firstCommand;
        addRequirements(firstCommand.getRequirements());
        addRequirements(secondCommand.getRequirements());

    }

    @Override
    public void initialize(){
        state = semiAutoStateState.firstCommand;
        currentCommand = firstCommand;
        currentCommand.initialize();
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
                currentCommand = firstCommand;
                currentCommand.initialize();
            }
        }

        if (state == semiAutoStateState.handoff){
            if (canHandoff.getAsBoolean()){
                state=semiAutoStateState.secondCommand;

                currentCommand = secondCommand;
                currentCommand.initialize();

                canFinishFlag=false;
            }
        }

        if (state == semiAutoStateState.secondCommand){
            if (cancelHandoff&&!canHandoff.getAsBoolean()){
                secondCommand.cancel();
                state = semiAutoStateState.handoff;
            }
        }

        currentCommand.execute();


        
    }

    @Override
    public boolean isFinished(){
        return state==semiAutoStateState.secondCommand && !secondCommand.isScheduled() && canFinishFlag;
    }

    @Override
    public void end(boolean wasInterrupted){
        currentCommand.end(wasInterrupted);
    }

}
