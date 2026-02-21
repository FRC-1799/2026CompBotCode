package frc.robot.commands.AutoStates;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.AutonConstants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.subsystems.GeneralManager;

public class IntakeHandoff extends SemiAutoState{
    public IntakeHandoff(){
        super(
            new DeferredCommand(
                    ()->{
                        return new ConditionalCommand(
                            SystemManager.swerve.driveToPose(SystemManager.getSwervePose().nearest(FieldPosits.intakingHandoffPoses), AutonConstants.intakeHandoffSpeed),
                            new InstantCommand(),
                            ()->FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
                        );
                    },
                Set.of(SystemManager.swerve)),
                GeneralManager.startIntaking()
        );
    }
}
