package frc.robot.subsystems.elevator;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class elevator extends SubsystemBase
{

  protected Distance setpoint = Meters.of(0);
  protected DigitalInput resetSwitch = new DigitalInput(Constants.elevatorConstants.resetSwitchID);
  private final TalonFX  elevatorMotor = new TalonFX(Constants.elevatorConstants.mainMotorID);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
      .withClosedLoopController(elevatorConstants.KP, elevatorConstants.KI, elevatorConstants.KD,elevatorConstants.maxSpeed, elevatorConstants.maxAccel)
      .withSoftLimit(Meters.of(-0.03), Meters.of(Constants.elevatorConstants.maxHeight))
      
      .withGearing(gearing(gearbox(5, 2)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ElevatorFeedforward(elevatorConstants.KS, elevatorConstants.KG, elevatorConstants.KV, elevatorConstants.KA))


      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new TalonFXWrapper(elevatorMotor,
                                                                            DCMotor.getKrakenX60(2),
                                                                            motorConfig);

  private final MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(0))
      .withMaxRobotLength(Meters.of(0))
      
      .withRelativePosition(Constants.elevatorConstants.fromRobotCenter);
  
  private ElevatorConfig m_config = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0))
      .withHardLimits(Meters.of(-0.03), Meters.of(Constants.elevatorConstants.maxHeight))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      
      .withMechanismPositionConfig(m_robotToMechanism)
      .withAngle(Constants.elevatorConstants.angle)
      .withMass(Pounds.of(16));
  
  private final Elevator m_elevator = new Elevator(m_config);


  protected TalonFX offMotor = new TalonFX(Constants.elevatorConstants.altMotorID);

  public elevator(){
    offMotor.setControl(new Follower(Constants.elevatorConstants.mainMotorID, true));
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
    motor.setPosition(setpoint);
    SmartDashboard.putNumber("elevatorHeight", getHeight());
    SmartDashboard.putBoolean("reset switch", !resetSwitch.get());
    if (!resetSwitch.get()&&getHeight()!=0){
      motor.setEncoderPosition(Meters.of(0));
    }
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }


  public Command sysId()
  {
    SignalLogger.start();
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

  public double getHeight(){
    return m_elevator.getHeight().in(Meters);
  }

  public double getHeightRender(){
    return getHeight()+Constants.elevatorConstants.elevatorIntakeEndOffset;
}

  public void setSetpoint(double newSetpoint){
    setpoint = Meters.of(newSetpoint);
  }

  public void setSetpoint(Distance newSetpoint){
    setpoint = newSetpoint;
  }

  public boolean isAtTop(){
    return getHeight()==Constants.elevatorConstants.maxHeight;
  }

    /**@return the 3d translation from the bottom of the elevator to the current point. all measurements use the rotation point of the wrist for consistency*/
  public Translation3d getTranslation(){
      return new Translation3d(getHeightRender()*Math.cos(Constants.elevatorConstants.angle.in(Radians)), 0, getHeightRender()*Math.sin(Constants.elevatorConstants.angle.in(Radians))).plus(Constants.elevatorConstants.fromRobotCenter);
  };

  public boolean isAtSetpoint(){
    return Math.abs(getHeight()-setpoint.in(Meters))<Constants.elevatorConstants.tolerance;
  }
}
