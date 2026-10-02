package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_DIAMETER;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_MASS;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Constants.shooterConstants.bottomMotorConstants;
import frc.robot.Constants.shooterConstants.topMotorConstants;
import frc.robot.FieldPosits;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
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
        
    public shooterState state = shooterState.rev;

    private final RobotPreferences pref = RobotPreferences.getInstance();

    int count  = 10;
    boolean indexerShouldBeOn=true;


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
    private final TalonFX beltMotor = new TalonFX(bottomMotorConstants.canID);

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

        SmartDashboard.putNumber("Shooter/ShooterTopSpeed", getTopFlywheelSpeed().in(RPM));
        SmartDashboard.putNumber("Shooter/ShooterBottomSpeed", getBottomFlywheelSpeed().in(RPM));
        SmartDashboard.putString("Shooter/ShooterState", state.toString());
        SmartDashboard.putNumber("Shooter/ShotDistance", SystemManager.getSwervePose().getTranslation().getDistance(FieldPosits.hubPose2d.getTranslation()));        
        SmartDashboard.putNumber("Shooter/BeltSpeed", beltFlyWheel.getSpeed().in(RPM));

        topShooter.updateTelemetry();
        //bottomShooter.updateTelemetry();
        
        if (state==shooterState.shooting){

            count--;
            if (count==0){
                indexerShouldBeOn=!indexerShouldBeOn;
                count = indexerShouldBeOn? 10: 2;
            }
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
    
    public void setVelocity() {
        if (pref.shootingSpeedRPM()<0.01) {
            topShooter.setDutyCycleSetpoint(0.0);
        } else {
            topShooter.setMechanismVelocitySetpoint(calculateTopSpinnerRpm());
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

    public Pose3d getHubPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                return FieldPosits.toRedAllicance(FieldPosits.hubPose3d);
            } else {
                return FieldPosits.hubPose3d;
            }
        }

        return FieldPosits.hubPose3d;
    }

    public AngularVelocity calculateTopSpinnerRpm() {
        var rpmPref = RobotPreferences.getInstance().shootingSpeedRPM();

        AngularVelocity launchAngularVelocity;

        if (rpmPref <= 1) {
            var robotVelocity = SystemManager.swerve.getFieldVelocity();
            var robotPose3d = new Pose3d(SystemManager.getSwervePose());
            var shooterPoseRel = pref.shooterPose();
            var pitchAngle = shooterPoseRel.getRotation().getMeasureY();
            var targetPoseRel = pref.targetRelativePose();

            var shooterPoseTransform = new Transform3d(shooterPoseRel.getTranslation(), shooterPoseRel.getRotation());
            var targetPoseTransform = new Transform3d(targetPoseRel.getTranslation(), targetPoseRel.getRotation());

            var shooterPose = robotPose3d.plus(shooterPoseTransform);
            var targetPose = getHubPose();

            var launchResult = calculateLaunch(
                    shooterPose,
                    targetPose.getTranslation(),
                    new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond),
                    pitchAngle,
                    Constants.fieldConstants.FUEL_BALL_MASS,
                    Constants.fieldConstants.FUEL_BALL_DIAMETER
            );

            launchAngularVelocity = launchResult.launchAngularVelocity(WHEEL_DIAMETER);
        } else {
            launchAngularVelocity = RevolutionsPerSecond.of(pref.shootingSpeedRPM());
        }

        return launchAngularVelocity;
    }

}
