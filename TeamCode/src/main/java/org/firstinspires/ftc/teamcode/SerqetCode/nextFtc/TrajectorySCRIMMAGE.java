package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

public class TrajectorySCRIMMAGE {
    public static double CalculateVelocity(double targetDistance) {
        // define values used for all calculations
        double g = -980.0; // cm/s^2 magnitude of acceleration due to gravity
        double GoalHeight = 55; // cm, target final height in goal
        double m = Math.min(-.5, (-(200.0 - GoalHeight)) / targetDistance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
        double a = (-GoalHeight/(targetDistance*targetDistance))+(m/targetDistance); // The "a" value in the parabola equation
        double b = ((2.0 * GoalHeight) / targetDistance) - m; // The "b" value in the parabola equation
        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
        //Degraded (hopefully) double launchVelocity = (1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a));
        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * (Math.sqrt(g / (2.0 * a))))* 1.77;  //1.8 little too strong, 1.6 too low, 2.6 is too high//12.21730476; //The launch velocity in cm/s

        return launchVelocity; // these are the target for the shooter's flywheel and servo angle adjustment
    }
    public static double CalculateAngle(double targetDistance) {
        // define values used for all calculations
        double g = -980.0; // cm/s^2 magnitude of acceleration due to gravity
        double GoalHeight = 55; // cm, target final height in goal
        double m = Math.min(-.5, (-(200.0 - GoalHeight)) / targetDistance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
        double a = (-GoalHeight/(targetDistance*targetDistance))+(m/targetDistance); // The "a" value in the parabola equation
        double b = ((2.0 * GoalHeight) / targetDistance) - m; // The "b" value in the parabola equation
        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
        //Degraded (hopefully) double launchVelocity = (1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a));
        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * (Math.sqrt(g / (2.0 * a))))* 1.23787177960363;  //12.21730476; //The launch velocity in cm/s

        double launchAngle = Math.toDegrees(rawlaunchAngle) * .00392157;
        // TODO - min/max filtering to remain within hardware limits ?
        return launchAngle;
    }; // these are the target for the shooter's flywheel and servo angle adjustments
}








