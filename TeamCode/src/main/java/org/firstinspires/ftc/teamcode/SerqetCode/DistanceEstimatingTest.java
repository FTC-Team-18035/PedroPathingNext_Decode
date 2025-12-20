package org.firstinspires.ftc.teamcode.SerqetCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Limelight3A AprilTag Distance TeleOp", group = "Sensor")
public class DistanceEstimatingTest extends LinearOpMode {

    private Limelight3A limelight;

    // Camera and target setup (meters)
    private static final double LIMELIGHT_HEIGHT = 0.5; //TODO 50 cm from floor
    private static final double TARGET_HEIGHT = 1.0;    //TODO 100 cm from floor
    private static final double LIMELIGHT_MOUNT_ANGLE = 25.0; //TODO degrees

    // Exponential smoothing factor
    private static final double ALPHA = 0.3;
    private Double smoothedDistance = null;

    @Override
    public void runOpMode() throws InterruptedException {

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

            if (result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (!fiducials.isEmpty()) {
                    LLResultTypes.FiducialResult tag = fiducials.get(0);

                    // Distance calculation using vertical angle
                    double verticalAngle = tag.getTargetYDegrees(); // degrees
                    double angleToTarget = LIMELIGHT_MOUNT_ANGLE + verticalAngle;
                    double distanceMeters = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(angleToTarget));
                    double distanceCm = distanceMeters * 100.0;

                    // Exponential smoothing
                    if (smoothedDistance == null) smoothedDistance = distanceCm;
                    else smoothedDistance = ALPHA * distanceCm + (1 - ALPHA) * smoothedDistance;

                    // Telemetry
                    telemetry.addData("Pipeline Index", limelight.getStatus().getPipelineIndex());
                    telemetry.addData("Tag ID", tag.getFiducialId());
                    telemetry.addData("Distance (cm)", "%.1f", smoothedDistance);
                    telemetry.addData("Vertical Angle (deg)", "%.1f", verticalAngle);
                    telemetry.addData("Horizontal Angle (deg)", "%.1f", tag.getTargetXDegrees());

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
