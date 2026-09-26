package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.*;

public class ProjectileCalculator {

    private static final double GRAVITY_METERS_SECOND = 9.80665;

    /**
     * Calculates the required launch velocity using WPILib unit types.
     *
     * @param launchHeight    The initial height from which the object is launched.
     * @param landingHeight   The final height where the object lands.
     * @param launchAngle     The launch angle relative to the horizontal.
     * @param landingDistance The horizontal distance traveled to the landing point.
     * @return The required initial launch velocity, or null if the target is physically unreachable.
     */
    public static LinearVelocity calculateLaunch(
            Distance launchHeight,
            Distance landingHeight,
            Angle launchAngle,
            Distance landingDistance) {

        // Convert all measurements to standard base units (Meters and Radians)
        double x = landingDistance.in(Meters);
        double deltaY = landingHeight.in(Meters) - launchHeight.in(Meters);
        double theta = launchAngle.in(Radians);

        // Pre-calculate trigonometric values
        double cosTheta = Math.cos(theta);
        double tanTheta = Math.tan(theta);

        // Calculate the denominator inside the square root
        double denominator = 2 * Math.pow(cosTheta, 2) * (x * tanTheta - deltaY);

        // If the denominator is less than or equal to 0, the target is physically unreachable
        if (denominator <= 0) {
            return null;
        }

        // Calculate the numerator
        double numerator = GRAVITY_METERS_SECOND * Math.pow(x, 2);

        // Compute velocity in meters per second
        double velocityValue = Math.sqrt(numerator / denominator);

        // Return as a WPILib LinearVelocity type
        return MetersPerSecond.of(velocityValue);
    }

    /**
     * Calculates where a projectile will land given independent launch and landing heights.
     *
     * @param v             Initial velocity in meters per second (m/s).
     * @param angle         Launch angle relative to the horizon in degrees.
     * @param launchHeight  Starting height above your reference ground level in meters (m).
     * @param landingHeight Target landing plane height above your reference ground level in meters (m).
     * @return A ProjectileResult containing flight time and horizontal range, or null if the apex is too low.
     */
    public static ProjectileResult calculateLanding(LinearVelocity v, Angle angle, Distance launchHeight, Distance landingHeight) {
        double angleRadians = angle.in(Radians);
        double v0 = v.in(MetersPerSecond);
        double launchHeightMeters = launchHeight.in(Meters);
        double landingHeightMeters = landingHeight.in(Meters);

        // Separate velocity components
        double vx = v0 * Math.cos(angleRadians);
        double vy0 = v0 * Math.sin(angleRadians);

        // Net vertical displacement needed (h0 relative to the landing plane)
        double relativeHeight = launchHeightMeters - landingHeightMeters;

        // Quadratic formula discriminant: vy0^2 + 2 * g * (y_start - y_end)
        double discriminant = (vy0 * vy0) + (2 * GRAVITY_METERS_SECOND * relativeHeight);

        // If discriminant is negative, the ball can never physically reach the landing height
        if (discriminant < 0) {
            return null;
        }

        // Calculate flight time
        double totalTime = (vy0 + Math.sqrt(discriminant)) / GRAVITY_METERS_SECOND;

        // Range = constant horizontal speed * time
        double landingDistance = vx * totalTime;

        return new ProjectileResult(Time.ofBaseUnits(totalTime, Seconds), Distance.ofBaseUnits(landingDistance, Meter));
    }

    /**
     * @param launchVelocity Relative muzzle speed needed (m/s)
     * @param yaw            Target robot-relative field orientation angle
     */
    public record LaunchResult(double launchVelocity, Rotation2d yaw) {

        @Override
            public String toString() {
                return String.format("Required Muzzle Velocity: %.3f m/s\nYaw Target: %.2f°",
                        launchVelocity, yaw.getDegrees());
            }
        }

    public record ProjectileResult(Time timeInAir, Distance landingDistance) {
    }
}
