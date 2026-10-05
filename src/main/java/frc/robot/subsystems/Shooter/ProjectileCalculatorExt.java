package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

public class ProjectileCalculatorExt {

    private static final double GRAVITY = 9.80665;
    private static final double AIR_DENSITY = 1.225;        // kg/m^3 (Standard sea level)
    private static final double DRAG_COEFFICIENT = 0.47;   // Smooth sphere standard
    private static final double MAX_VELOCITY_LIMIT = 50.0; // m/s safety bound
    private static final double TIME_STEP = 0.005;         // 5ms iteration accuracy
    private static final double ERROR_TOLERANCE_METERS = 0.02;

    public record LaunchResult(LinearVelocity launchVelocity, Rotation2d launcherYaw, double error) {

        public static final LaunchResult kZero = new LaunchResult(MetersPerSecond.of(0.0), Rotation2d.kZero, 0.0);

        // Multiply this by 2 if shooter only has one flywheel pinching against a bar
        public AngularVelocity launchAngularVelocity(Distance wheelDiameter) {
            return ProjectileCalculatorCommon.calculateAngularVelocity(launchVelocity, wheelDiameter);
        }
    }

    /**
     * @param launcherPose Pose of the launcher in field space (rotation is ignored)
     * @param targetLocation Target location relative to the launcher (rotation is ignored)
     * @param platformVelocity Velocity of the platform in field space
     * @param pitchAngle Angle of the launcher's pitch from the ground
     * @param ballMass Mass of the ball
     * @param ballDiameter Diameter of the ball
     * @return LaunchResult
     */
    public static LaunchResult calculateLaunch(Pose3d launcherPose,
                                               Translation3d targetLocation,
                                               Translation2d platformVelocity,
                                               Angle pitchAngle,
                                               Mass ballMass,
                                               Distance ballDiameter) {

        // 1. Convert WPILib strongly typed wrappers into SI standard units
        double startX = launcherPose.getX();
        double startY = launcherPose.getY();
        double startZ = launcherPose.getZ();

        double targetX = targetLocation.getX();
        double targetY = targetLocation.getY();
        double targetZ = targetLocation.getZ();

        double vPlatformX = platformVelocity.getX();
        double vPlatformY = platformVelocity.getY();

        double pitchRad = pitchAngle.in(Radians);
        double massKg = ballMass.in(Kilograms);
        double radiusM = ballDiameter.in(Meters) / 2.0;

        // 2. Pre-calculate continuous 3D drag factors
        double crossSectionalArea = Math.PI * Math.pow(radiusM, 2);
        double dragFactorK = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * crossSectionalArea;

        // Base targeted geometric direction
        double dx = targetX - startX;
        double dy = targetY - startY;
        double targetDistance2D = Math.hypot(dx, dy);

        if (targetDistance2D == 0) {
            throw new IllegalArgumentException("Target cannot sit vertically aligned on the launcher axis.");
        }

        // 3. Multi-Variable Numerical Boundary Search
        double lowV = 0.1;
        double highV = MAX_VELOCITY_LIMIT;

        double optimizedV = -1.0;
        double optimizedYawRad = Math.atan2(dy, dx); // Base heading initialization

        // Outer solver loop to converge on perfect velocity magnitude
        for (int vIter = 0; vIter < 30; vIter++) {
            double midV = (lowV + highV) / 2.0;

            double lowYaw = optimizedYawRad - Math.PI / 3;
            double highYaw = optimizedYawRad + Math.PI / 3;
            double currentTestYaw = optimizedYawRad;
            double finalSimZ = startZ;

            boolean hitHorizontalPlane = false;

            for (int yawIter = 0; yawIter < 20; yawIter++) {
                currentTestYaw = (lowYaw + highYaw) / 2.0;

                // Relative 3D vectors
                double vRelX = midV * Math.cos(pitchRad) * Math.cos(currentTestYaw);
                double vRelY = midV * Math.cos(pitchRad) * Math.sin(currentTestYaw);
                double vRelZ = midV * Math.sin(pitchRad);

                // Absolute Ground-Relative Initial Velocity
                double vx = vRelX + vPlatformX;
                double vy = vRelY + vPlatformY;
                double vz = vRelZ;

                // Simulation initial markers
                double simX = startX;
                double simY = startY;
                double simZ = startZ;

                int steps = 0;
                int maxSteps = 2000; // Hard loop escape safety (10 seconds of max simulated flight time)
                boolean simulationRunning = true;

                while (simulationRunning && steps < maxSteps) {
                    steps++;
                    double currentSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);

                    // Apply continuous aerodynamic decelerations
                    double ax = -(dragFactorK / massKg) * currentSpeed * vx;
                    double ay = -(dragFactorK / massKg) * currentSpeed * vy;
                    double az = -GRAVITY - ((dragFactorK / massKg) * currentSpeed * vz);

                    // Integrate spatial steps
                    simX += vx * TIME_STEP + 0.5 * ax * TIME_STEP * TIME_STEP;
                    simY += vy * TIME_STEP + 0.5 * ay * TIME_STEP * TIME_STEP;
                    simZ += vz * TIME_STEP + 0.5 * az * TIME_STEP * TIME_STEP;

                    vx += ax * TIME_STEP;
                    vy += ay * TIME_STEP;
                    vz += az * TIME_STEP;

                    // FIXED: Terminate strictly based on horizontal radial range boundary crossing
                    // from the launch origin to avoid infinite vector tracking alignment lock
                    double curDistance2D = Math.hypot(simX - startX, simY - startY);
                    if (curDistance2D >= targetDistance2D || simZ < (targetZ - 5.0)) {
                        finalSimZ = simZ;

                        // 1. Calculate the vector from the starting position to the simulated landing spot
                        double simDx = simX - startX;
                        double simDy = simY - startY;

                        // 2. Compute the perpendicular cross-product to determine if the simulated point
                        // is to the left or right of the true target line-of-sight vector (dx, dy)
                        double crossProduct = (dx * simDy) - (dy * simDx);

                        if (Math.abs(crossProduct) / targetDistance2D < 0.01) {
                            hitHorizontalPlane = true;
                        } else if (crossProduct > 0) {
                            highYaw = currentTestYaw; // Overshot to the left
                        } else {
                            lowYaw = currentTestYaw;  // Undershot to the right
                        }
                        simulationRunning = false;
                    }
                }
                if (hitHorizontalPlane) break;
            }

            // Once cross-track displacement is minimized, verify vertical elevation tracking error
            if (finalSimZ < targetZ) {
                lowV = midV;  // Ball landed short/low, increase velocity floor
            } else {
                highV = midV; // Ball flew over the target height ceiling
                optimizedV = midV;
                optimizedYawRad = currentTestYaw;

                if (Math.abs(finalSimZ - targetZ) <= ERROR_TOLERANCE_METERS) {
                    break;
                }
            }
        }

        double error = 0;
        if (optimizedV < 0 || optimizedV >= (MAX_VELOCITY_LIMIT - 0.1)) {
            error = 1;
        }

        return new LaunchResult(
                MetersPerSecond.of(optimizedV),
                new Rotation2d(optimizedYawRad),
                error
        );
    }

    /**
     * @param launcherPose Pose of the launcher in field space (rotation is ignored)
     * @param launchVelocity Velocity of the projectile in robot space
     * @param launcherYaw Angle of the launcher's in field space
     * @param platformVelocity Velocity of the platform in field space
     * @param pitchAngle Angle of the launcher's pitch from the ground
     * @param ballMass Mass of the ball
     * @param ballDiameter Diameter of the ball
     * @param targetFloorHeight Height of the target from the ground
     * @return Landing location of the projectile
     */
    public static Translation3d calculateLanding(
            Pose3d launcherPose,
            LinearVelocity launchVelocity,
            Rotation2d launcherYaw,
            Translation2d platformVelocity,
            Angle pitchAngle,
            Mass ballMass,
            Distance ballDiameter,
            Distance targetFloorHeight) {

        // 1. Convert strongly-typed parameters to standard SI metrics
        double startX = launcherPose.getX();
        double startY = launcherPose.getY();
        double startZ = launcherPose.getZ();

        double vMag = launchVelocity.in(MetersPerSecond);
        double yawRad = launcherYaw.getRadians();
        double pitchRad = pitchAngle.in(Radians);

        double vPlatformX = platformVelocity.getX();
        double vPlatformY = platformVelocity.getY();

        double massKg = ballMass.in(Kilograms);
        double radiusM = ballDiameter.in(Meters) / 2.0;
        double targetZ = targetFloorHeight.in(Meters);

        // 2. Pre-calculate the aerodynamic cross-sectional drag coefficient constant
        double crossSectionalArea = Math.PI * Math.pow(radiusM, 2);
        double dragFactorK = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * crossSectionalArea;

        // 3. Compute relative vector velocities mapping out of the launcher frame
        double vRelX = vMag * Math.cos(pitchRad) * Math.cos(yawRad);
        double vRelY = vMag * Math.cos(pitchRad) * Math.sin(yawRad);
        double vRelZ = vMag * Math.sin(pitchRad);

        // 4. Transform relative velocity to absolute ground-relative values
        double vx = vRelX + vPlatformX;
        double vy = vRelY + vPlatformY;
        double vz = vRelZ;

        // 5. Initialize tracking positions
        double simX = startX;
        double simY = startY;
        double simZ = startZ;

        int steps = 0;
        int maxSteps = 2000; // 10-second maximum flight duration safeguard

        // Determine if we are firing upward to a higher target or downward
        boolean firingUpward = targetZ > startZ;

        while (steps < maxSteps) {
            steps++;
            double currentSpeed = Math.sqrt(vx * vx + vy * vy + vz * vz);

            // Compute decelerations from continuous 3D drag vectors
            double ax = -(dragFactorK / massKg) * currentSpeed * vx;
            double ay = -(dragFactorK / massKg) * currentSpeed * vy;
            double az = -GRAVITY - ((dragFactorK / massKg) * currentSpeed * vz);

            // Numerical integration steps
            simX += vx * TIME_STEP + 0.5 * ax * TIME_STEP * TIME_STEP;
            simY += vy * TIME_STEP + 0.5 * ay * TIME_STEP * TIME_STEP;
            simZ += vz * TIME_STEP + 0.5 * az * TIME_STEP * TIME_STEP;

            vx += ax * TIME_STEP;
            vy += ay * TIME_STEP;
            vz += az * TIME_STEP;

            // Termination criteria check
            if (firingUpward) {
                // For high targets, break when the ball passes peak apex and drops back below target height
                if (vz < 0 && simZ <= targetZ) {
                    break;
                }
            } else {
                // For low targets, break as soon as the ball drops below the landing plane height
                if (simZ <= targetZ) {
                    break;
                }
            }

            // Extreme failure fallback escape
            if (simZ < (targetZ - 5.0)) {
                break;
            }
        }

        return new Translation3d(simX, simY, simZ);
    }
}




