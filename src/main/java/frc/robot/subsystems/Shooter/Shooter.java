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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;
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

      private final TalonFX                   ShooterMotor    = new TalonFX(shooterConstants.mainMotorID);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0, RPM.of(10000), RPM.per(Second).of(60))
      .withIdleMode(MotorMode.COAST)
      .withGearing(1)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withFollowers(Pair.of(new TalonFX(shooterConstants.followerMotorID),false));
  


    private final SmartMotorController motor         = new TalonFXWrapper(ShooterMotor, DCMotor.getKrakenX60(2), motorConfig);

  private final FlyWheelConfig       shooterConfig = new FlyWheelConfig(motor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-5000), RPM.of(5000))
      
      .withSpeedometerSimulation(RPM.of(7500));

      
  private final FlyWheel             shooter       = new FlyWheel(shooterConfig);
        


    @Override
    public void periodic(){

        SmartDashboard.putNumber("Shooter/ShooterSpeed", getFlywheelSpeed().in(RPM));
        SmartDashboard.putString("Shooter/ShooterState", state.toString());
        
        
        shooter.updateTelemetry();

    }
    
    @Override
    public void simulationPeriodic(){
        shooter.simIterate();
    }
    

    public void startRevving(){
        state = shooterState.rev;
        shooter.setSpeed(shooterConstants.shootingSpeed).schedule();
    }

    public void startShooting(){
        state=shooterState.shooting;
        shooter.setSpeed(shooterConstants.shootingSpeed).schedule();
    }

    public void stop(){
        if (state==shooterState.resting) rest();
        else startRevving();;
    }

    public void rest(){
        state = shooterState.resting;
        shooter.set(0).schedule();
    }

    public AngularVelocity getFlywheelSpeed(){
        return shooter.getSpeed();
    }



}
