package frc.robot.subsystems.vision.calibration;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.vision.LimelightHelpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;

public class MathCal {

    public static double WidthDistance2Arc(double width, double distance) {
        return 2 *  Math.atan((width / 2) / distance) * (180 / Math.PI);
    }

//    var camTranslate = new Translation3d(f[2], f[0] * -1, f[1]);
//            var camRot = new Rotation3d(Degrees.of(f[5] * -1), Degrees.of(f[3]), Degrees.of(f[4] * -1));

    public static Pose3d Camera2RobotSpace(Pose3d targetPose) {

        var ct = targetPose.getTranslation();
        var cr = targetPose.getRotation();

        var tt = new Transform3d();

        var rp = new Pose3d(
                new Translation3d(ct.getZ(), ct.getX() * -1, ct.getY() * -1),
                new Rotation3d(cr.getZ() + Math.PI, cr.getX(), cr.getY())
        );

        return rp;
    }

    public static Pose3d LocalPlus(Angle x, Angle y, Angle z, Pose3d pose) {
        var r = pose.getRotation();
        return new Pose3d(
                pose.getTranslation(),
                new Rotation3d(
                        r.getX() + x.in(Radian),
                        r.getY() + y.in(Radian),
                        r.getZ() + z.in(Radian)
                )
        );
    }


    public static Vector<N3> VectorCameraForward() { return VecBuilder.fill(0,0,1); }
    public static Vector<N3> VectorCameraUp() { return VecBuilder.fill(0,1,0); }
    public static Vector<N3> VectorCameraRight() { return VecBuilder.fill(1,0,0); }

    public static Rotation3d LookAtRotation(Translation3d observerPosition, Translation3d targetPosition, Vector<N3> observerDirection) {
        // Calculate the relative vector (direction from observer to target)
        var dirTranslation = targetPosition.minus(observerPosition);

        // Get a rotation by using the direction of the observer and the vector to the target
        return new Rotation3d(observerDirection, dirTranslation.toVector());
    }

    public static Pose3d RelativeToPose(Pose3d pose, Pose3d target) {
        var p = new Pose3d(target.getTranslation().plus(pose.getTranslation()), pose.getRotation());
        p = p.rotateAround(target.getTranslation(), target.getRotation());
        return p;
    }

//    public static Translation3d Rotation2Point(Pose3d p1, Pose3d p2) {
//        var dir = p1.getTranslation().minus(p2.getTranslation());
//        var r = dir.
//
////        var mag = p1.getTranslation().getDistance(p2.getTranslation());
////        var dirNorm = dir;
////        if(mag != 0) {
////            dirNorm = new Translation3d(dir.getX() / mag, dir.getY() / mag, dir.getZ() / mag);
////        }
////
////        return dirNorm;
//        //return new Rotation3d(dirNorm.getX(), dirNorm.getY(), dirNorm.getZ());
//    }
}
