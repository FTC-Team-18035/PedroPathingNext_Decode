package org.firstinspires.ftc.teamcode.SerqetCode;

public class Trajectory {
    public static double[] Calculate(double targetDistance) {
        double distance = targetDistance;
        // define values used for all calculations
        double g = 980.0; // cm/s^2 magnitude of acceleration due to gravity
        double y0 = 53.0; // cm, target final height in goal
        double m = Math.min(-1.5, (-200.0 - y0) / distance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
        double a = -(y0 / Math.pow(distance, 2)) + (m / distance); // The "a" value in the parabola equation
        double b = ((2.0 * y0) / distance) - m; // The "b" value in the parabola equation
        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
        //Degraded (hopefully) double launchVelocity = (1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a));
        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a)))*12.21730476; //The launch velocity in cm/s

        double launchAngle = Math.toDegrees(rawlaunchAngle) * .00392157;
                                             // TODO - min/max filtering to remain within hardware limits ?
        return new double[] { launchVelocity, launchAngle }; // these are the target for the shooter's flywheel and servo angle adjustments
    }
}







