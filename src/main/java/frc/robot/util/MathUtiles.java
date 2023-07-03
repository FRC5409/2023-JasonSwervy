package frc.robot.util;

/**
 * @author Alexander Szura
 */
public class MathUtiles {
    /**
     * Get the target rotation angle, in radians.
     * 
     * 0 deg = forward
     * 90 deg = left
     * 180 deg = backward
     * 270 deg = right
     * 
     * @param xRotation right stick x
     * @param yRotation right stick y
     * @return target angle
     */
    public static double getRotation(double xRotation, double yRotation) {

        if (xRotation == 0 && yRotation == 0)
            return -1;
        
        double angle = Math.atan2(-xRotation, -yRotation);
        if (angle < 0) angle += Math.toRadians(360);
        return angle;
    }

    /**
     * Snaps a value to a multiple of that value
     * @param value the value to snap
     * @param snap the snapping value
     * @return the new snapped value
     */
    public static double snapTo(double value, double snap) {
        return Math.round(value / snap) * snap;
    }
}
