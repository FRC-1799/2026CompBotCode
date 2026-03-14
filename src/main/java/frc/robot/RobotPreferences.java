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

    public static final String beltFeedSpeed = "Belt Feed Speed";

    public static final String shooterSpeedRPM = "top Shooting Speed RPM";



    private static void registerDoublePreference(String key, double val){
        if (!Preferences.containsKey(key)) Preferences.initDouble(key, val);

    }


    private RobotPreferences() {

        registerDoublePreference(bottomShootingSpeedDutyCycle, 0.6);
        registerDoublePreference(topShootingSpeedDutyCycle, 0.7);
        registerDoublePreference(intakeIngestSpeed, -0.2);
        registerDoublePreference(intakeBackwardSpeed, 0.2);
        registerDoublePreference(beltFeedSpeed, 0.2);
        registerDoublePreference(shooterSpeedRPM, 3000);
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
