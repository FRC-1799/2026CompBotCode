package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldPosits;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import frc.robot.Utils.time.DeltaTimeRolling;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_DIAMETER;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorCommon.calculateLinearVelocity;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

public class CalculateLaunchWorker {

    private static CalculateLaunchWorker instance;
    private final ReadWriteLock rwLock = new ReentrantReadWriteLock();
    private final RobotPreferences pref = RobotPreferences.getInstance();
    Thread worker = null;
    private ProjectileCalculatorExt.LaunchResult launchPrediction = ProjectileCalculatorExt.LaunchResult.kZero;
    private DoublePublisher launchCalcDeltaTimeMs;
    private DoublePublisher launchCalcRpm;
    private DoublePublisher launchCalcInError;

    //
    // Singleton
    //
    private CalculateLaunchWorker() {
    }

    public static CalculateLaunchWorker getInstance() {
        if (instance == null) {
            instance = new CalculateLaunchWorker();
        }
        return instance;
    }

    //
    // Calculate launch
    //
    public ProjectileCalculatorExt.LaunchResult getLaunchPrediction() {
        rwLock.readLock().lock();
        try {
            return launchPrediction;
        } finally {
            rwLock.readLock().unlock();
        }
    }

    public void Start() {
        if (launchCalcDeltaTimeMs == null) {
            var nt = NetworkTableInstance.getDefault().getTable("SmartDashboard/LaunchCalc");
            launchCalcDeltaTimeMs = nt.getDoubleTopic("DeltaTimeAvrMs").publish();
            launchCalcInError = nt.getDoubleTopic("Error").publish();
            launchCalcRpm = nt.getDoubleTopic("Predicted Rpm").publish();
        }

        if (worker == null || !worker.isAlive()) {
            worker = new Thread(this::CalculateLaunchWorker);
            worker.start();
        }
    }

    public void Stop() {
        worker.interrupt();
    }

    private void CalculateLaunchWorker() {
        DataLogManager.log("CalculateLaunchWorker: Started");

        var deltaTime = new DeltaTimeRolling(10);
        int updateCount = 0;
        while (!Thread.currentThread().isInterrupted()) {
            ProjectileCalculatorExt.LaunchResult result;

            boolean override = false;
            deltaTime.Start();
            {
                var rpmPref = RobotPreferences.getInstance().shootingSpeedRPM();
                override = rpmPref < 0;

                Pose3d robotPose;
                ChassisSpeeds robotSpeed;

                if (RobotBase.isReal()) {
                    robotPose = new Pose3d(SystemManager.getSwervePose());
                    robotSpeed = SystemManager.swerve.getFieldVelocity();
                } else {
                    robotPose = new Pose3d(SystemManager.getRealPoseMaple());
                    robotSpeed = SystemManager.swerve.getFieldVelocity();
                }

                if (override) {
                    result = calculateLaunchParameters(robotPose, robotSpeed);
                } else {
                    var robotRotaton = ProjectileCalculatorCommon.rotateTowardTarget(robotPose.toPose2d(), getHubPose().toPose2d());

                    result = new ProjectileCalculatorExt.LaunchResult(
                            calculateLinearVelocity(
                                    RPM.of(rpmPref),
                                    WHEEL_DIAMETER),
                            robotRotaton,
                            0.0);
                }

                rwLock.writeLock().lock();
                try {
                    launchPrediction = result;
                    launchCalcInError.set(result.error());
                } finally {
                    rwLock.writeLock().unlock();
                }
            }
            // rolling average requires calling this
            var deltaMs = deltaTime.DeltaMilliSec();
            if (updateCount-- < 0) {
                launchCalcDeltaTimeMs.set(deltaMs);
                launchCalcRpm.set(Math.round(result.launchAngularVelocity(WHEEL_DIAMETER).times(2).in(RPM)));
                updateCount = 10;
            }

            // we'll chill for a second if manual override
            var sleepTime = override ? 250 : 50;
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                // drop because we're checking for thread interrupt in loop
            }
        }

        DataLogManager.log("CalculateLaunchWorker: Stopped");
    }

    public ProjectileCalculatorExt.LaunchResult calculateLaunchParameters(Pose3d robotPose3d, ChassisSpeeds robotVelocity) {
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
                Constants.fieldConstants.FUEL_BALL_DIAMETER,
                pref.launchDragCoeff()
        );

        return launchResult;
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
}
