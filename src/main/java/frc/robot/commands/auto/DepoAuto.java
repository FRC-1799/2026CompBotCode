package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.subsystems.AutoManager;
import frc.robot.subsystems.GeneralManager;

public class DepoAuto extends SequentialCommandGroup{
    public DepoAuto(){
        super(
            SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.depo, new Rotation2d(
                SystemManager.getSwervePose().getX()-FieldPosits.depo.getX(),
                SystemManager.getSwervePose().getY()-FieldPosits.depo.getY()))
            ),
            new WaitCommand(1),
            AutoManager.startShooting(),

            GeneralManager.startIntaking(),
            SystemManager.swerve.driveToPose(new Pose2d(FieldPosits.mid, new Rotation2d(
                SystemManager.getSwervePose().getX()-FieldPosits.mid.getX(),
                SystemManager.getSwervePose().getY()-FieldPosits.mid.getY()))
            ),
            AutoManager.startShooting()


        );
    }
}
