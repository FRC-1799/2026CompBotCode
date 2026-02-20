package frc.robot.commands.AutoStates;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.swervedrive.smallAutoDrive;
import frc.robot.subsystems.GeneralManager;

public class PassingHandoff extends SemiAutoState{
    public PassingHandoff(){
        super(new DeferredCommand(
                ()->{return new smallAutoDrive(
                    new Pose2d(
                        SystemManager.getSwervePose().getTranslation(),
                        utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), SystemManager.getSwervePose().nearest(FieldPosits.passingPoses))
                    )
                    );
                },
                Set.of(SystemManager.swerve)),
            GeneralManager.startPassing());
    }
}
