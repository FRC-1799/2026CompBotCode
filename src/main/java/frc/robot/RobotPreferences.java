package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Utils.utilFunctions;

/** 
 * Preferences that can be accessed from Elastic, Smart Dashboard, etc. 
 * */
public class RobotPreferences {
    private static RobotPreferences instance = null;
    public static final String preferenceReset = "Preference Reset";
    public static final String topShootingSpeedDutyCycle = "Top Shooting Speed Duty Cycle";
    public static final String bottomShootingSpeedDutyCycle = "Bottom Shooting Speed Duty Cycle";
    public static final String intakeIngestSpeed = "Intake Ingest Speed";
    public static final String intakeBackwardSpeed = "Intake Backward Speed";

    public static final String beltFeedSpeed = "Bottom Shooting Speed Duty Cycle";

    public static final String shooterSpeedRPM = "top Shooting Speed RPM";






    private RobotPreferences() {
        if (!Preferences.containsKey(bottomShootingSpeedDutyCycle)) Preferences.initDouble(bottomShootingSpeedDutyCycle, 0.6);
        if (!Preferences.containsKey(topShootingSpeedDutyCycle)) Preferences.initDouble(topShootingSpeedDutyCycle, 0.7);
        if (!Preferences.containsKey(intakeIngestSpeed)) Preferences.initDouble(intakeIngestSpeed, -0.2);
        if (!Preferences.containsKey(intakeBackwardSpeed)) Preferences.initDouble(intakeBackwardSpeed, 0.2);
        if (!Preferences.containsKey(beltFeedSpeed)) Preferences.initDouble(beltFeedSpeed, 0.2);
        if (!Preferences.containsKey(shooterSpeedRPM)) Preferences.initDouble(shooterSpeedRPM, 3000);

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

    public double shootingSpeedRPM() {
        return Preferences.getDouble(shooterSpeedRPM, 3000);
    }

    public double intakeBackwardSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(intakeBackwardSpeed, 0.2), -1.0, 1.0);
    }

    public double beltFeedSpeed() {
        return utilFunctions.clamp(Preferences.getDouble(beltFeedSpeed, 0.2), -1.0, 1.0);
    }



    /** 
     * Call this to get an instance of the preferences 
     * */
    public static RobotPreferences getInstance() {
        if (instance == null) {
            instance = new RobotPreferences();
        }
        return instance;
    }

}
