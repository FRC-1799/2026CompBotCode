package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Constants.shooterConstants;
import frc.robot.Utils.utilFunctions;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class simShooter extends Shooter{

        
    protected double cooldown=0;
    protected double matchTime = DriverStation.getMatchTime();


    
        @Override
        public void periodic(){
            super.periodic();
            cooldown-=Timer.getFPGATimestamp()-matchTime;
            matchTime=Timer.getFPGATimestamp();
            if (state==shooterState.shooting){
                if (cooldown<=0){
                    shootInternal();
                    cooldown=0.2;
                }
            }


        }
    
    
    public void shootInternal(){
            
        if (SystemManager.intake.getPieceCount()>0){
            SystemManager.intake.removePiece();
            
            SimulatedArena.getInstance()
            .addGamePieceProjectile(new RebuiltFuelOnFly(
                // Obtain robot position from drive simulation
                SystemManager.getRealPoseMaple().getTranslation(),
                // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                new Translation2d(),
                // Obtain robot speed from drive simulation
                SystemManager.swerve.getFieldVelocity(),
                // Obtain robot facing from drive simulation
                SystemManager.getRealPoseMaple().getRotation(),
                // The height at which the fuel is ejected
                shooterConstants.shooterHeight,
            // The initial speed of the fuel
            MetersPerSecond.of(getFlywheelSpeed().in(RPM)*shooterConstants.SimRPMToMPS),
            // The fuel is ejected at a 35-degree slope
            shooterConstants.shotAngle
            ));
            

        }
    }


}
