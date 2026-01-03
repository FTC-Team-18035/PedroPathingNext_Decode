package org.firstinspires.ftc.teamcode.SerqetCode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "Limelight3A AprilTag Distance TeleOp", group = "Sensor")
public class DistanceEstimatingTest extends LinearOpMode {

    private Limelight3A limelight;

    // Camera and target setup (meters)
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13; //15.0;

    // Exponential smoothing factor
    private static final double ALPHA = 0.3;
    private Double smoothedDistance = null;

    private static double Kp = -.015;//-0.05;//-0.1;
    private static double minCommand = .09;//0.07; //was .05

    private double left_command, right_command;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;



    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        // Start with pipeline 8 (AprilTag)
        limelight.pipelineSwitch(8);
        limelight.start();

        telemetry.addLine("Limelight 3A AprilTag TeleOp Ready");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            // Constants

            // Default steering
            double steeringAdjust = 0.0;

            if (gamepad1.y && result != null && result.isValid()) { // Button 9 ≈ Y (change if needed)

                // Limelight horizontal offset (tx equivalent)
                double tx = result.getTx(); // degrees

                double headingError = -tx;

                if (Math.abs(headingError) > 1.0) {
                    if (headingError < 0) {
                        steeringAdjust = Kp * headingError + minCommand;
                    } else {
                        steeringAdjust = Kp * headingError - minCommand;
                    }
                }
                left_command  = steeringAdjust;
                right_command = -steeringAdjust;

                frontRight.setPower(right_command);
                backRight.setPower(right_command);
                frontLeft.setPower(left_command);
                backLeft.setPower(left_command);
            }

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);

                    // Distance calculation using vertical angle
                    double verticalAngle = Math.toRadians(tag.getTargetYDegrees()); // degrees
                    double angleToTarget = LIMELIGHT_MOUNT_ANGLE + verticalAngle;
                    double distanceMeters = .5170 / Math.tan(Math.toRadians(result.getTy() + LIMELIGHT_MOUNT_ANGLE)); // (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
                    double distanceCm = distanceMeters * 100.0;

                    // Exponential smoothing
                    if (smoothedDistance == null) smoothedDistance = distanceCm;
                    else smoothedDistance = ALPHA * distanceCm + (1 - ALPHA) * smoothedDistance;

                    // Telemetry
                    telemetry.addData("Pipeline Index", limelight.getStatus().getPipelineIndex());
                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("Distance (cm)", "%.1f", distanceCm);
                    telemetry.addData("Vertical Angle (deg)", "%.1f", verticalAngle);
                    telemetry.addData("Horizontal Angle (deg)", "%.1f", tag.getTargetXDegrees());
                    telemetry.addData("Target Y", result.getTy());
                    telemetry.addData("Motor power", right_command);

                } else {
                    telemetry.addLine("No AprilTags detected");
                    smoothedDistance = null; // reset smoothing buffer
                }

            } else {
                telemetry.addLine("No Limelight data available");
                smoothedDistance = null;
            }

            telemetry.update();
        }

        limelight.stop();
    }
}
