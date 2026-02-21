package frc.robot.commands.auto;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.subsystems.GeneralManager;

public class MidGrab extends SequentialCommandGroup{
    public MidGrab(){
        super(
            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),
            new ShootHandoff(),
     
            GeneralManager.startIntaking(),
            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),
            new ShootHandoff()
        );

    }
}
