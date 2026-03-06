package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldPosits;
import frc.robot.SystemManager;
import frc.robot.Utils.utilFunctions;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.bottomMotorConstants;
import frc.robot.Constants.shooterConstants.topMotorConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public abstract class Shooter extends SubsystemBase{
    public enum shooterState{
        resting, 
        rev,
        shooting
    }
        
    public shooterState state = shooterState.rev;

    private final TalonFX topShooterMotor = new TalonFX(topMotorConstants.canID);
    private final TalonFX bottomShooterMotor = new TalonFX(bottomMotorConstants.canID);


    private final SmartMotorControllerConfig topMotorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(topMotorConstants.P, topMotorConstants.I, topMotorConstants.D, RPM.of(10000), RPM.per(Second).of(1000))
      .withIdleMode(MotorMode.COAST)
      .withGearing(topMotorConstants.gearReduction)
      .withTelemetry("TopShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(60))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(topMotorConstants.shooterFeedForward)
      .withSimFeedforward(topMotorConstants.shooterFeedForward)
      .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorControllerConfig bottomMotorConfig = new SmartMotorControllerConfig(new Subsystem() {
        
    })
      .withClosedLoopController(bottomMotorConstants.P, bottomMotorConstants.I, bottomMotorConstants.D, RPM.of(10000), RPM.per(Second).of(1000))
      .withIdleMode(MotorMode.COAST)
      .withGearing(bottomMotorConstants.gearReduction)
      .withTelemetry("BottomShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(60))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(bottomMotorConstants.shooterFeedForward)
      .withSimFeedforward(bottomMotorConstants.shooterFeedForward)
      .withControlMode(ControlMode.CLOSED_LOOP);
  
  


    private final SmartMotorController topMotor = new TalonFXWrapper(topShooterMotor, DCMotor.getKrakenX60(1), topMotorConfig);
    private final SmartMotorController bottomMotor = new TalonFXWrapper(bottomShooterMotor, DCMotor.getKrakenX60(1), bottomMotorConfig);


    private final FlyWheelConfig topShooterConfig = new FlyWheelConfig(topMotor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("TopFlywheelMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-6600), RPM.of(6600))
      
      .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheelConfig bottomShooterConfig = new FlyWheelConfig(bottomMotor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("TopFlywheelMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-6600), RPM.of(6600))
      
      .withSpeedometerSimulation(RPM.of(7500));

      
    private final FlyWheel topShooter = new FlyWheel(topShooterConfig);
    private final FlyWheel bottomShooter = new FlyWheel(bottomShooterConfig);
    
    private final TalonFX indexer = new TalonFX(shooterConstants.beltMotorID);
        


    @Override
    public void periodic(){

        SmartDashboard.putNumber("Shooter/ShooterTopSpeed", getTopFlywheelSpeed().in(RPM));
        SmartDashboard.putNumber("Shooter/ShooterBottomSpeed", getBottomFlywheelSpeed().in(RPM));
        SmartDashboard.putString("Shooter/ShooterState", state.toString());
        SmartDashboard.putNumber("Shooter/ShotDistance", SystemManager.getSwervePose().getTranslation().getDistance(FieldPosits.hubPose2d.getTranslation()));        
        
        topShooter.updateTelemetry();
        bottomShooter.updateTelemetry();
        
        if (state==shooterState.shooting){
            indexer.set(shooterConstants.indexerShootSpeed);
        }
        else{
            indexer.set(shooterConstants.indexerStopSpeed);
        }

    }
    
    @Override
    public void simulationPeriodic(){
        topShooter.simIterate();
        bottomShooter.simIterate();
    }
    

    public void startRevving(){
        state = shooterState.rev;
        topShooter.set(topMotorConstants.shootingSpeedDutyCycle).schedule();

        bottomShooter.set(bottomMotorConstants.shootingSpeedDutyCycle).schedule();


    }

    public void startShooting(){
        state=shooterState.shooting;
        topShooter.set(topMotorConstants.shootingSpeedDutyCycle).schedule();
        bottomShooter.set(bottomMotorConstants.shootingSpeedDutyCycle).schedule();
    }

    public void stop(){
        if (state==shooterState.resting) rest();
        else startRevving();;
    }

    public void rest(){
        state = shooterState.resting;
        bottomShooter.set(0).schedule();
        topShooter.set(0).schedule();

    }

    public AngularVelocity getTopFlywheelSpeed(){
        return topShooter.getSpeed();
    }

    public AngularVelocity getBottomFlywheelSpeed(){
        return bottomShooter.getSpeed();
    }

    public Pose2d getClosestShootPoint(){
        Pose2d robotPose = SystemManager.getSwervePose();
        Pose2d goalPose = FieldPosits.hubPose2d;

        Transform2d dif = robotPose.minus(goalPose);

        
        return new Pose2d(
            goalPose.plus(dif.div(Math.hypot(dif.getX(), dif.getY())).times(shooterConstants.shootRadius)).getTranslation(),
            utilFunctions.getAngleBetweenTwoPoints(SystemManager.getSwervePose(), FieldPosits.hubPose2d));
    }

}
