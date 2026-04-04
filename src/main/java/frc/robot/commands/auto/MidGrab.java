package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.commands.AutoStates.ShootHandoff;
import frc.robot.subsystems.GeneralManager;



public class MidGrab extends SequentialCommandGroup{

    protected static boolean leftSide;

    public MidGrab(){
        super(
            new DeferredCommand(
                ()->{
                    return new InstantCommand(
                        ()->{
                            leftSide = 
                            utilFunctions.getDistanceBetweenTwoPoints(SystemManager.getSwervePose(), FieldPosits.leftSideAutoHandoff).in(Meters) <
                            utilFunctions.getDistanceBetweenTwoPoints(SystemManager.getSwervePose(), FieldPosits.rightSideAutoHandoff).in(Meters);
                        }
                    );
                }, 
                Set.of()
            ),
            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),

            SystemManager.swerve.driveToPose(leftSide? FieldPosits.leftSideAutoHandoff : FieldPosits.rightSideAutoHandoff, MetersPerSecond.of(2)),
            new ShootHandoff(),
     
            GeneralManager.intaking(),
            new DeferredCommand(
                ()->{
                    return SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                        SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                        SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
                    );
                },
                Set.of(SystemManager.swerve)
            ),
            SystemManager.swerve.driveToPose(leftSide? FieldPosits.leftSideAutoHandoff : FieldPosits.rightSideAutoHandoff, MetersPerSecond.of(2)),

            new ShootHandoff()
        );

    }
}
