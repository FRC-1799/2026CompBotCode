package frc.robot.commands.states;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Utils.utilFunctions;
import frc.robot.subsystems.GeneralManager;

public class shooting extends Command{


    public shooting(){
        addRequirements(GeneralManager.subsystems);
        //addRequirements(SystemManager.swerve);
    }

    @Override 
    public void initialize(){
        SystemManager.shooter.startShooting();

    }


    @Override
    public void end(boolean wasCanceled){
        SystemManager.shooter.stop();
        GeneralManager.endCallback(wasCanceled);
    }
}
