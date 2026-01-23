package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
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

public class simShooter extends SubsystemBase{
    public enum shooterState{
        resting, 
        rev,
        shooting
    }
        
    protected double cooldown=0;
    protected double matchTime = DriverStation.getMatchTime();

    public shooterState state = shooterState.resting;


    protected Pose3d currentGoal=FieldPosits.hubPose3d;
    public StructPublisher<Rotation2d> rotationGoalPublisher = NetworkTableInstance.getDefault().getStructTopic("shooter/RotationGoal", Rotation2d.struct).publish();
    public BooleanPublisher isShootingPublisher = NetworkTableInstance.getDefault().getBooleanTopic("shooter/isShooting").publish();
        
    public static final double g = 9.8; 
    
        @Override
        public void periodic(){
            cooldown-=Timer.getFPGATimestamp()-matchTime;
            matchTime=Timer.getFPGATimestamp();
            if (state==shooterState.shooting){
                if (cooldown<=0){
                    shootInternal();
                    cooldown=0.2;
                }
            }

            rotationGoalPublisher.set(new Rotation2d( getShotAngle(utilFunctions.getDistanceBetweenTwoPoints(
                SystemManager.getSwervePose(), currentGoal.toPose2d()),
                MetersPerSecond.of(8),
                currentGoal.getMeasureZ().minus(shooterConstants.shooterHeight)
            )));
            isShootingPublisher.set(state==shooterState.shooting);

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
            MetersPerSecond.of(8),
            // The fuel is ejected at a 35-degree slope
            getShotAngle(utilFunctions.getDistanceBetweenTwoPoints(
                SystemManager.getSwervePose(), currentGoal.toPose2d()),
                MetersPerSecond.of(8),
                currentGoal.getMeasureZ().minus(shooterConstants.shooterHeight)
            )));
            

        }
    }

    public void startShooting(){
        state=shooterState.shooting;
    }

    public void stop(){
        state = shooterState.resting;
    }
    

    public Angle getShotAngle(Distance range, LinearVelocity fireSpeed, Distance heightDif){
        if (!RobotBase.isReal()){
            //Stolen from https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)

            double v = fireSpeed.in(MetersPerSecond);
            double x = range.in(Meters);
            double y = heightDif.in(Meters);
            return Radians.of(
                Math.atan2(
                    Math.pow(v, 2)+ 
                    Math.sqrt(
                        Math.pow(v, 4)-
                        g*(g*x*x+2*y*v*v)
                    ),
                g*x));
        }

        return Radians.of(0);
    }

    public void setGoal(Pose3d goal){
        currentGoal=goal;
    }

    public Pose3d getGoal(){
        return currentGoal;
    }

}
