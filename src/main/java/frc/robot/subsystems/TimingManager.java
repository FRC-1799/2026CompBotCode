package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Timing Manager that is able to return whether our alliance's hub is 
 * active or not. This is done by finding the auto winner and active shift
 * based on the time elapsed in the match. 
*/
public class TimingManager {


    protected enum Shift {
        AUTO(0, 20, ActiveType.BOTH),
        TRANSITION(20, 30, ActiveType.BOTH),
        SHIFT_1(30, 55, ActiveType.AUTO_LOSER),
        SHIFT_2(55, 80, ActiveType.AUTO_WINNER),
        SHIFT_3(80, 105, ActiveType.AUTO_LOSER),
        SHIFT_4(105, 130, ActiveType.AUTO_WINNER),
        ENDGAME(130, 160, ActiveType.BOTH),
        NA(-1, -1, ActiveType.BOTH);

        final int startTime;
        final int endTime;
        final ActiveType activeType;

        private Shift(int startTime, int endTime, ActiveType activeType) {
            this.startTime = startTime;
            this.endTime = endTime;
            this.activeType = activeType;
        }
    }

    protected enum ActiveType {
        BOTH,
        AUTO_WINNER,
        AUTO_LOSER
    }

    protected static Optional<Alliance> getAutoWinner() {
        String msg = DriverStation.getGameSpecificMessage(); 
        char msgChar = msg.length() > 0 ? msg.charAt(0) : ' '; // the first charecter is the alliance that lost auto stage
        switch (msgChar) {
            case 'B':
                return Optional.of(Alliance.Blue);
            case 'R':
                return Optional.of(Alliance.Red);
            default:
                return Optional.empty();
        }
    }

    protected static double getMatchTime() {
        if (DriverStation.isFMSAttached()){
            if (DriverStation.isAutonomous()) {
                if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
                return 20 - DriverStation.getMatchTime(); // Subtracts from 20 so that timer counts up instead of down

            } else if (DriverStation.isTeleop()) {
                if (DriverStation.getMatchTime() < 0) return DriverStation.getMatchTime();
                return 160 - DriverStation.getMatchTime();
            }
            return -1;
        }
        else {
            return Timer.getFPGATimestamp();
        }
        
    }

    public static Shift getCurrentShift() {
        double matchTime = getMatchTime();
        if (matchTime < 0) return Shift.NA;

        for (Shift shift : Shift.values()) {
            if (shift.startTime < matchTime && matchTime < shift.endTime) { 
                return shift;
            }
        }
        return Shift.NA;
    }



    public static boolean isActive() {
        Optional<Alliance> autoWinner = getAutoWinner();
        Shift currentShift = getCurrentShift();
        Optional<Alliance> currentAlliance = DriverStation.getAlliance();

            switch (currentShift.activeType) {
            case BOTH:
                return true;

            case AUTO_WINNER:
                return autoWinner.isPresent() && autoWinner.get().toString() == currentAlliance.toString(); 

            case AUTO_LOSER:
                return autoWinner.isPresent() && autoWinner.get().toString() != currentAlliance.toString();
            
            default:
                // Will not run unless a new value is added to ActiveType
                return false;
        }
    }

    public static double timeRemaining() {
        double time = getCurrentShift().endTime;
        if (time > 0) {
            return time - getMatchTime();
        }
        else {
            return -1;
        }
        

    }

    public static void periodic() {
        SmartDashboard.putBoolean("Timer/IsActive", isActive());
        SmartDashboard.putNumber("Timer/TimeRemainingInStep", timeRemaining());
    }
}



