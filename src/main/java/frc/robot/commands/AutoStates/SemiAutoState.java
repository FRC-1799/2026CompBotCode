package frc.robot.commands.AutoStates;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.SystemManager;

public class SemiAutoState extends SequentialCommandGroup{
    Command driveCommand;
    Command handOffCommand;
    BooleanSupplier canHandoff;

    public SemiAutoState (Command driveCommand){
        super(driveCommand);
        this.driveCommand = driveCommand;
    }

    public SemiAutoState(Command driveCommand, Command handoffCommand){
        super(driveCommand, new InstantCommand(()->SystemManager.swerve.lock()), handoffCommand);
        this.driveCommand=driveCommand;
        this.handOffCommand=handoffCommand;
    }

    public SemiAutoState(Command driveCommand, Command handoffCommand, BooleanSupplier canHandoff){
        super(driveCommand, new InstantCommand(()->SystemManager.swerve.lock()), new WaitUntilCommand(canHandoff), handoffCommand);
        this.driveCommand=driveCommand;
        this.handOffCommand=handoffCommand;
        this.canHandoff=canHandoff;
    }
}
