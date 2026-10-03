package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.limelightConstants;
import frc.robot.Constants.shooterConstants;
import frc.robot.Utils.utilFunctions;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.shooterConstants.shooterRobotRelativePose;
import static frc.robot.subsystems.Shooter.ProjectileCalculatorExt.calculateLaunch;

/**
 * Preferences that can be accessed from Elastic, Smart Dashboard, etc.
 *
 */
public class RobotPreferences {
    public static final String preferenceReset = "Preference Reset";
    public static final String topShootingSpeedDutyCycle = "Top Shooting Speed Duty Cycle";
    public static final String bottomShootingSpeedDutyCycle = "Bottom Shooting Speed Duty Cycle";
    public static final String intakeIngestSpeed = "Intake Ingest Speed";
    public static final String intakeBackwardSpeed = "Intake Backward Speed";
    public static final String beltFeedSpeed = "Belt Feed RPM";
    public static final String indexerSpeed = "Indexer RPM";
    public static final String shooterSpeedRPM = "Shooter/top Shooting Speed RPM";
    public static final String shooterPose = "Shooter/Relative Location and angle of shooter";
    public static final String targetRelativePose = "Shooter/Distance of target Relative to pose";
    public static final String limelight1Pose = "Limelight 1 Pose";
    public static final String limelight2Pose = "Limelight 2 Pose";
    public static final String aimbotRadius = "Aimbot Radius";
    public static final String gameControllerAxisMag = "Game Controller Axis Magnitude";
    private static RobotPreferences instance = null;


    private RobotPreferences() {

        registerDoublePreference(bottomShootingSpeedDutyCycle, 0.6);
        registerDoublePreference(topShootingSpeedDutyCycle, 0.7);
        registerDoublePreference(intakeIngestSpeed, -0.2);
        registerDoublePreference(intakeBackwardSpeed, 0.2);
        registerDoublePreference(beltFeedSpeed, 120);
        registerDoublePreference(indexerSpeed, 60);
        registerDoublePreference(shooterSpeedRPM, 0);
        registerPose3dPreference(shooterPose, shooterRobotRelativePose);
        registerPose3dPreference(targetRelativePose);
        registerPose3dPreference(limelight1Pose, limelightConstants.limelight1Pose);
        registerPose3dPreference(limelight2Pose, limelightConstants.limelight2Pose);
        registerDoublePreference(aimbotRadius, shooterConstants.shootRadius);
        registerDoublePreference(aimbotRadius, shooterConstants.shootRadius);
        registerDoublePreference(gameControllerAxisMag, 4.0);
    }

    public static void registerPose3dPreference(String key, Pose3d val) {
        registerDoublePreference(key + "/Roll", val.getRotation().getMeasureX().in(Degrees));
        registerDoublePreference(key + "/Pitch", val.getRotation().getMeasureY().in(Degrees));
        registerDoublePreference(key + "/Yaw", val.getRotation().getMeasureZ().in(Degrees));
        registerDoublePreference(key + "/X", val.getX());
        registerDoublePreference(key + "/Y", val.getY());
        registerDoublePreference(key + "/Z", val.getZ());
    }

    public static void registerPose3dPreference(String key) {
        registerPose3dPreference(key, Pose3d.kZero);
    }

    private static void registerDoublePreference(String key, double val) {
        if (!Preferences.containsKey(key)) Preferences.initDouble(key, val);

    }

    /**
     * Call this to get an instance of the preferences
     *
     */
    public static RobotPreferences getInstance() {
        if (instance == null) {
            instance = new RobotPreferences();
        }
        return instance;
    }

    public Pose3d getPose3dPreference(String key) {
        return new Pose3d(
                new Translation3d(
                        Preferences.getDouble(key + "/X", 0),
                        Preferences.getDouble(key + "/Y", 0),
                        Preferences.getDouble(key + "/Z", 0)
                ),
                new Rotation3d(
                        Degrees.of(Preferences.getDouble(key + "/Roll", 0)),
                        Degrees.of(Preferences.getDouble(key + "/Pitch", 0)),
                        Degrees.of(Preferences.getDouble(key + "/Yaw", 0))
                )
        );
    }

    public double bottomShootingSpeedDutyCycle() {
        return utilFunctions.clamp(Preferences.getDouble(bottomShootingSpeedDutyCycle, 0.6), -1.0, 1.0);
    }

    public double topShootingSpeedDutyCycle() {
        return utilFunctions.clamp(Preferences.getDouble(topShootingSpeedDutyCycle, 0.7), -1.0, 1.0);
    }

    public double intakeIngestSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(intakeIngestSpeed, 0.2), -1.0, 1.0);
    }

    public double beltFeedSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(beltFeedSpeed, 120.0), 10.0, 500.0);
    }

    public double indexerSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(intakeIngestSpeed, 60), 10, 500);
    }

    public double shootingSpeedRPM() {
        return utilFunctions.clamp(Preferences.getDouble(shooterSpeedRPM, 0), -1.0, 5000);
    }


    public Pose3d shooterPose() {
        return getPose3dPreference(shooterPose);
    }

    public Pose3d targetRelativePose() {
        return getPose3dPreference(targetRelativePose);
    }

    public double intakeBackwardSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(intakeBackwardSpeed, 0.2), -1.0, 1.0);
    }

    public double aimbotRadius() {
        return Preferences.getDouble(aimbotRadius, shooterConstants.shootRadius);
    }

    public Pose3d getLimelight1Pose() {
        return getPose3dPreference(limelight1Pose);
    }

    public Pose3d getLimelight2Pose() {
        return getPose3dPreference(limelight2Pose);
    }

    public double gameControllerAxisMag() {
        return utilFunctions.clamp(Preferences.getDouble(gameControllerAxisMag, 4.0), 1.0, 10.0);
    }

}
