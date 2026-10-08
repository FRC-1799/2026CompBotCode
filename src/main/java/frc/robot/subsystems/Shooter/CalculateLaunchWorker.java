package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants;
import frc.robot.FieldPosits;
import frc.robot.RobotPreferences;
import frc.robot.SystemManager;
import frc.robot.Utils.time.DeltaTime;
import frc.robot.Utils.time.DeltaTimeRolling;

import java.util.Optional;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.shooterConstants.topMotorConstants.WHEEL_DIAMETER;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorCommon.calculateLinearVelocity;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

public class CalculateLaunchWorker {

    private RobotPreferences pref = RobotPreferences.getInstance();

    private final ReadWriteLock rwLock = new ReentrantReadWriteLock();
    Thread worker;
    private ProjectileCalculatorExt.LaunchResult launchPrediction = ProjectileCalculatorExt.LaunchResult.kZero;

    private DoublePublisher launchCalcDeltaTimeMs;
    private DoublePublisher launchCalcInError;

    //
    // Singleton
    //
    private CalculateLaunchWorker() {}

    private static CalculateLaunchWorker instance;
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
        if(launchCalcDeltaTimeMs == null) {
            var nt = NetworkTableInstance.getDefault().getTable("SmartDashboard");
            launchCalcDeltaTimeMs = nt.getDoubleTopic("LaunchPredict DeltaTimeAvrMs").publish();
            launchCalcInError = nt.getDoubleTopic("LaunchPredict Error").publish();
        }

        worker = new Thread(this::CalculateLaunchWorker);
        worker.start();
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

            deltaTime.Start();
            {
                if (RobotBase.isReal()) {
                    result = calculateLaunchParameters(
                            new Pose3d(SystemManager.getSwervePose()),
                            SystemManager.swerve.getFieldVelocity());
                } else {
                    result = calculateLaunchParameters(
                            new Pose3d(SystemManager.getRealPoseMaple()),
                            SystemManager.swerve.getFieldVelocity());
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
            if(updateCount-- < 0) {
                launchCalcDeltaTimeMs.set(deltaMs);
                updateCount = 10;
            }

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                // drop because we're checking for thread interrupt in loop
            }
        }

        DataLogManager.log("CalculateLaunchWorker: Stopped");
    }

    public ProjectileCalculatorExt.LaunchResult calculateLaunchParameters(Pose3d robotPose3d, ChassisSpeeds robotVelocity) {
        var rpmPref = RobotPreferences.getInstance().shootingSpeedRPM();

        AngularVelocity launchAngularVelocity;

        if (rpmPref < 0) {
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

            return launchResult;
        } else {
            return new ProjectileCalculatorExt.LaunchResult(calculateLinearVelocity(RPM.of(rpmPref), WHEEL_DIAMETER), Rotation2d.fromRadians(0), 0.0);
        }
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
