package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class utilFunctions {


    /**
     * calculates the Hypotenuse for a triangle with specified x and y sides
     * @param x the length of the x side
     * @param y the length of the y side
     * @return the length of the Hypotenuse 
     */
    public static double pythagorean(double x, double y){
        return Math.sqrt(Math.pow(x, 2)+ Math.pow(y, 2));
    }

    /**
     * calculates the distance between two specified coordinate points
     * @param x1 the x coord of the first point
     * @param x2 the x coord of the second point
     * @param y1 the y coord of the first point
     * @param y2 the y coord of the second point
     * @return the distance between the two points
     */
    public static double pythagorean(double x1, double x2, double y1, double y2){
        return pythagorean(x1-x2, y1-y2);
    }

    public static boolean[][] flipBoolArray(boolean[][] toFlip){
        boolean[][]returnArr = new boolean[toFlip[0].length][toFlip.length];
        for (int i=0; i<toFlip.length; i++){
            for ( int j=0; j<returnArr.length; j++){
                returnArr[j][i] = toFlip[i][j];
            }
        }
        return returnArr;
    }

    public static Rotation2d getAngleBetweenTwoPoints(Pose2d firstPoint, Pose2d secondPoint){
        return new Rotation2d(Math.atan2(firstPoint.getY()-secondPoint.getY(), firstPoint.getX()-secondPoint.getX())).rotateBy(Rotation2d.fromDegrees(180));
    }
}
