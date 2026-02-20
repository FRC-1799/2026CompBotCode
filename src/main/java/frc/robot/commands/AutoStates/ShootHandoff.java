package frc.robot.commands.AutoStates;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class ShootHandoff extends SemiAutoState{
    public ShootHandoff(){
        super(
            new DeferredCommand(
                ()->{return new ConditionalCommand(
                    SystemManager.swerve.driveToPose(new Pose2d(
                        SystemManager.getSwervePose().nearest(FieldPosits.scoringPoses).getTranslation(),
                        utilFunctions.getAngleBetweenTwoPoints(
                            new Pose2d(SystemManager.getSwervePose().nearest(FieldPosits.scoringPoses).getTranslation(), new Rotation2d()),
                            FieldPosits.hubPose2d)

                    )),
                     SystemManager.swerve.driveToPose(new Pose2d(SystemManager.getSwervePose().getTranslation(), utilFunctions.getAngleBetweenTwoPoints(new Pose2d(SystemManager.getSwervePose().getTranslation(), new Rotation2d()), FieldPosits.hubPose2d))),
                    ()->!FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())
                );},
                Set.of(SystemManager.swerve)),
            new SequentialCommandGroup(GeneralManager.startShooting(), new WaitUntilCommand(()->SystemManager.intake.getPieceCount()==0))
        );
    }
}
