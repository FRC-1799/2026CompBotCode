package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_DIAMETER;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_MASS;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorCommon.calculateLinearVelocity;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants.bottomMotorConstants;
import frc.robot.Constants.shooterConstants.topMotorConstants;
import frc.robot.FieldPosits;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import frc.robot.Utils.DebugUtil;
import frc.robot.Utils.RollingAverage;
import frc.robot.Utils.utilFunctions;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public abstract class Shooter extends SubsystemBase{
    public enum shooterState{
        resting, 
        rev,
        shooting
    }
        
    public shooterState state = shooterState.resting;

    private final RobotPreferences pref = RobotPreferences.getInstance();

    int count  = 10;
    boolean indexerShouldBeOn=true;

    RollingAverage topShooterRpmAverage = new RollingAverage(50);


    //
    // Top Flywheel
    //
    private final TalonFX topShooterMotor = new TalonFX(topMotorConstants.canID);

    private final SmartMotorControllerConfig topShooterMotorConfig = new SmartMotorControllerConfig(this)
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

    private final SmartMotorController topMotorWrapper = new TalonFXWrapper(topShooterMotor, DCMotor.getKrakenX60(1), topShooterMotorConfig);

    private final FlyWheelConfig topShooterFlyWheelConfig = new FlyWheelConfig(topMotorWrapper)
      .withDiameter(WHEEL_DIAMETER)
      .withMass(WHEEL_MASS)
      .withTelemetry("TopShooterFlywheel", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-6600), RPM.of(6600))
      .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheel topShooter = new FlyWheel(topShooterFlyWheelConfig);


    //
    // Bottom Indexer
    //
    private final TalonFX indexerMotor = new TalonFX(bottomMotorConstants.canID);

    private final SmartMotorControllerConfig indexerMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(topMotorConstants.P, topMotorConstants.I, topMotorConstants.D, RPM.of(10000), RPM.per(Second).of(1000))
            .withIdleMode(MotorMode.COAST)
            .withGearing(topMotorConstants.gearReduction)
            .withTelemetry("BottomIndexerMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(topMotorConstants.shooterFeedForward)
            .withSimFeedforward(topMotorConstants.shooterFeedForward)
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController indexerWrapper = new TalonFXWrapper(indexerMotor, DCMotor.getKrakenX60(1), indexerMotorConfig);

    private final FlyWheelConfig indexerFlyWheelConfig = new FlyWheelConfig(indexerWrapper)
            .withDiameter(WHEEL_DIAMETER)
            .withMass(WHEEL_MASS)
            .withTelemetry("BottomIndexerFlywheel", TelemetryVerbosity.HIGH)
            .withSoftLimit(RPM.of(-6600), RPM.of(6600))
            .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheel indexerFlyWheel = new FlyWheel(indexerFlyWheelConfig);


    //
    // Belt Motor
    //
    private final TalonFX beltMotor = new TalonFX(Constants.shooterConstants.beltMotorID);

    private final SmartMotorControllerConfig beltMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(topMotorConstants.P, topMotorConstants.I, topMotorConstants.D, RPM.of(10000), RPM.per(Second).of(1000))
            .withIdleMode(MotorMode.COAST)
            .withGearing(topMotorConstants.gearReduction)
            .withTelemetry("BeltMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(60))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(topMotorConstants.shooterFeedForward)
            .withSimFeedforward(topMotorConstants.shooterFeedForward)
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController beltMotorWrapper = new TalonFXWrapper(beltMotor, DCMotor.getKrakenX60(1), beltMotorConfig);

    private final FlyWheelConfig beltMotorFlyWheelConfig = new FlyWheelConfig(beltMotorWrapper)
            .withDiameter(WHEEL_DIAMETER)
            .withMass(WHEEL_MASS)
            .withTelemetry("BeltFlywheel", TelemetryVerbosity.HIGH)
            .withSoftLimit(RPM.of(-6600), RPM.of(6600))
            .withSpeedometerSimulation(RPM.of(7500));

    private final FlyWheel beltFlyWheel = new FlyWheel(beltMotorFlyWheelConfig);



    @Override
    public void periodic(){

        DebugUtil.Publish("Shooter State", state.toString());

        SmartDashboard.putNumber("Shooter/ShooterTopSpeed", topShooterRpmAverage.getAverage(Math.round(getTopFlywheelSpeed().in(RPM))));
        SmartDashboard.putNumber("Shooter/ShooterBottomSpeed",  getBottomFlywheelSpeed().in(RPM));
        SmartDashboard.putString("Shooter/ShooterState", state.toString());
        SmartDashboard.putNumber("Shooter/ShotDistance", SystemManager.getSwervePose().getTranslation().getDistance(FieldPosits.hubPose2d.getTranslation()));        
        SmartDashboard.putNumber("Shooter/BeltSpeed", beltFlyWheel.getSpeed().in(RPM));

        topShooter.updateTelemetry();
        //bottomShooter.updateTelemetry();


        if (state==shooterState.shooting){
            if (indexerShouldBeOn){
                beltFlyWheel.setMechanismVelocitySetpoint(RPM.of(pref.beltFeedSpeed()));
                indexerFlyWheel.setMechanismVelocitySetpoint(RPM.of(pref.indexerSpeed()));
            }
            else{
                beltFlyWheel.setDutyCycleSetpoint(0.0);
                indexerFlyWheel.setDutyCycleSetpoint(0.0);
            }
        }
        else{
            beltFlyWheel.setDutyCycleSetpoint(0.0);
            indexerFlyWheel.setDutyCycleSetpoint(0.0);
        }
    }
    
    @Override
    public void simulationPeriodic(){
        topShooter.simIterate();
    }

    int setvCount = 0;
    public void setVelocity() {
        if(state != shooterState.resting) {
            var launchPredict = SystemManager
                    .calculateLaunchWorker
                    .getLaunchPrediction();

            if (launchPredict.error() > 0.0) return;

            var rpm = launchPredict.launchAngularVelocity(WHEEL_DIAMETER);

            if (rpm.in(RPM) < 0.01) {
                topShooter.setDutyCycleSetpoint(0.0);
            } else {
                var wheelspeed = rpm.times(2);
                topShooter.setMechanismVelocitySetpoint(wheelspeed);
                DebugUtil.Publish("TopRPM", wheelspeed.in(RPM));
                DebugUtil.Publish("Count", setvCount++);
            }
        }
    }

    public void startRevving(){
        state = shooterState.rev;
        setVelocity();
    }

    public void startShooting(){
        state=shooterState.shooting;
        setVelocity();
    }

    //TODO: This function needs to be fixed
    public void stop(){
        if (state==shooterState.resting) rest();
        else startRevving();
    }

    public void rest(){
        state = shooterState.resting;
        topShooter.setDutyCycleSetpoint(0.0);
    }

    public AngularVelocity getTopFlywheelSpeed(){
        return topShooter.getSpeed();
    }

    public AngularVelocity getBottomFlywheelSpeed(){
        return RPM.of(0);
    }

    public Command sysId() {return topShooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}

    public Pose2d getClosestShootPoint(){

        Pose2d robotPose;


        if (!FieldPosits.alianceZone.contains(SystemManager.getSwervePose().getTranslation())){
            robotPose = SystemManager.getSwervePose().nearest(FieldPosits.trenches);
        }
        else{
            robotPose = SystemManager.getSwervePose();
        }

        Pose2d goalPose = FieldPosits.hubPose2d;

        Transform2d dif = robotPose.minus(goalPose);

        
        return new Pose2d(
            goalPose.plus(dif.div(Math.hypot(dif.getX(), dif.getY())).times(RobotPreferences.getInstance().aimbotRadius())).getTranslation(),
            utilFunctions.getAngleBetweenTwoPoints(robotPose, FieldPosits.hubPose2d));
    }

    public boolean hasPiecesRemaining(){
        return false; 
    }


}
