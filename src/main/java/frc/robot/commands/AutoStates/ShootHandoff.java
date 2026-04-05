package frc.robot.commands.AutoStates;

import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class ShootHandoff extends SemiAutoState{
    public ShootHandoff(){
        this(()->true);
    }

    public ShootHandoff(BooleanSupplier canHandoff){
        this(canHandoff, false);
    }


    public ShootHandoff(BooleanSupplier canHandoff, boolean midShootCancel){
        super(

            new DeferredCommand(()->{return SystemManager.swerve.driveToPose(SystemManager.shooter.getClosestShootPoint());},
                Set.of(SystemManager.swerve)),
            new SequentialCommandGroup(GeneralManager.shooting(), new WaitUntilCommand(()->!SystemManager.shooter.hasPiecesRemaining())),
            canHandoff
        );
    }

    
}
