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
    


    private RobotPreferences() {
        if (!Preferences.containsKey(preferenceReset)) Preferences.initBoolean(preferenceReset, false);

        if (Preferences.getBoolean(preferenceReset, false)) {
            Preferences.removeAll();
        }

        if (!Preferences.containsKey(preferenceReset)) Preferences.initBoolean(preferenceReset, false);

        if (!Preferences.containsKey(bottomShootingSpeedDutyCycle)) Preferences.initDouble(bottomShootingSpeedDutyCycle, 0.6); // 6️⃣
        if (!Preferences.containsKey(topShootingSpeedDutyCycle)) Preferences.initDouble(topShootingSpeedDutyCycle, 0.7);       // 7️⃣
        


    }

    public double bottomShootingSpeedDutyCycle() {
        return utilFunctions.clamp(Preferences.getDouble(bottomShootingSpeedDutyCycle, 0.6), -1.0, 1.0);
    }

    public double topShootingSpeedDutyCycle() {
        return utilFunctions.clamp(Preferences.getDouble(topShootingSpeedDutyCycle, 0.7), -1.0, 1.0);
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
