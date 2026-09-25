package frc.robot.subsystems.Shooter;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ProjectileCalculator {

    private static final double GRAVITY_METERS_SECOND = 9.80665;
    private static final double AIR_DENSITY_KG_M3 = 1.225; // Standard sea-level air density
    private static final double TIME_STEP_SECONDS = 0.001; // 1 millisecond step size for integration precision

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
     * Calculates where a projectile will land, accounting for quadratic air friction.
     *
     * @param v             Initial velocity.
     * @param angle         Launch angle relative to the horizon.
     * @param launchHeight  Starting height above reference ground level.
     * @param landingHeight Target landing plane height above reference ground level.
     * @param mass          Mass of the projectile object.
     * @param crossArea     Cross-sectional area of the projectile (m²).
     * @param dragCoeff     Drag coefficient (Cd) based on shape (e.g., ~0.47 for a sphere, ~0.3 for a sleek note).
     * @return A ProjectileResult containing flight time and horizontal range, or null if it immediately falls below landing plane.
     */
    public static ProjectileResult calculateLandingWithDrag(
            LinearVelocity v,
            Angle angle,
            Distance launchHeight,
            Distance landingHeight,
            Mass mass,
            double crossArea,
            double dragCoeff) {

        double v0 = v.in(MetersPerSecond);
        double angleRadians = angle.in(Radians);
        double m = mass.in(Kilograms);

        // Initial positions and velocities
        double x = 0.0;
        double y = launchHeight.in(Meters);
        double targetY = landingHeight.in(Meters);

        double vx = v0 * Math.cos(angleRadians);
        double vy = v0 * Math.sin(angleRadians);

        double totalTime = 0.0;

        // Base exit constraint if it starts below or on the landing plane moving down
        if (y < targetY || (y == targetY && vy <= 0)) {
            return null;
        }

        // Drag constant multiplier factor: 0.5 * rho * Cd * A
        double dragFactor = 0.5 * AIR_DENSITY_KG_M3 * dragCoeff * crossArea;

        // Numerical Integration loop (runs until projectile crosses the target height moving downward)
        while (y > targetY || vy > 0) {
            double speed = Math.hypot(vx, vy);

            // Drag force magnitude: Fd = dragFactor * v²
            double forceDrag = dragFactor * speed * speed;

            // Deconstruct drag forces opposite to the velocity vectors
            double dragForceX = -forceDrag * (vx / speed);
            double dragForceY = -forceDrag * (vy / speed);

            // Compute net acceleration: a = F_net / m
            double ax = dragForceX / m;
            double ay = -GRAVITY_METERS_SECOND + (dragForceY / m);

            // Update positions using current velocity (Euler-Cromer)
            x += vx * TIME_STEP_SECONDS;
            y += vy * TIME_STEP_SECONDS;

            // Update velocities using calculated acceleration
            vx += ax * TIME_STEP_SECONDS;
            vy += ay * TIME_STEP_SECONDS;

            totalTime += TIME_STEP_SECONDS;

            // Safety timeout to prevent infinite loop if conditions become unresolvable
            if (totalTime > 15.0) {
                return null;
            }
        }

        return new ProjectileResult(
                Time.ofBaseUnits(totalTime, Seconds),
                Distance.ofBaseUnits(x, Meter)
        );
    }

    public record ProjectileResult(Time timeInAir, Distance landingDistance) {
    }
}
