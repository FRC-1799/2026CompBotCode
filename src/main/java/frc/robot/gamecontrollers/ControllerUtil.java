package frc.robot.gamecontrollers;

import edu.wpi.first.math.MathUtil;

import java.util.Optional;

public class ControllerUtil {

    public static Optional<Double> getJoystickToHeading(double x, double y, double deadzone) {
        // 1. Calculate magnitude (distance from center)
        double magnitude = Math.hypot(x, y);

        // 2. Return 0 if the stick is inside the deadzone
        if (magnitude < deadzone) {
            return Optional.empty();
        }

        // 3. Invert Y and calculate radians (-PI to PI)
        double correctedY = -y;
        double radian = Math.atan2(correctedY, x);
        return Optional.of(MathUtil.angleModulus(radian));
    }

}
