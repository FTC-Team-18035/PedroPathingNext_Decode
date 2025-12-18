package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

    @TeleOp(name = "Limelight_Pinpoint")
    public class Limelight_PinPoint_FIXED extends OpMode {

        private Limelight3A limelight;
        private GoBildaPinpointDriver pinpoint;
        private double llDistance;

        @Override
        public void init() {
            // Get references from hardwareMap but do not call methods yet
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8); // April Tag 20 pipeline
        }

        @Override
        public void start() {
            // Optional: short delay to allow Pinpoint v2 to initialize
            try {
                Thread.sleep(100);  // 100ms
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            // Configure the odometry
            configurePinpoint();

            // Set initial robot position
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

            // Start the limelight
            limelight.start();
        }

        @Override
        public void loop() {
            // Update the robot's position
            pinpoint.update();
            Pose2D pose2D = pinpoint.getPosition();

            // Update limelight orientation
            limelight.updateRobotOrientation(pose2D.getHeading(AngleUnit.DEGREES));

            // Read limelight results
            LLResult llResult = limelight.getLatestResult();
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                llDistance = getDistance(llResult.getTa());
                telemetry.addData("Calculated Distance", llDistance);
                telemetry.addData("Target X", llResult.getTx());
                telemetry.addData("Target Area", llResult.getTa());
                telemetry.addData("Botpose", botPose.toString());
            }

            telemetry.addData("pinpoint x", pinpoint.getPosition().getX(DistanceUnit.MM));
            telemetry.addData("pinpoint y", pinpoint.getPosition().getY(DistanceUnit.MM));
            telemetry.addData("pinpoint heading", pinpoint.getPosition().getHeading(AngleUnit.DEGREES));

            telemetry.update();
        }

        private double getDistance(double ta) {
            double distance = Math.pow(4931.375/ta, 0.60233) ; //based on mycurvefit equation, solved for distance
            return distance;
        }

        private void configurePinpoint() {
            /*
             * Set the odometry pod offsets relative to the robot's tracking center
             * X pod offset: left of center positive, right of center negative
             * Y pod offset: forward positive, backward negative
             */
            pinpoint.setOffsets(-94.9706, -70.70786, DistanceUnit.MM);

            /*
             * Set the kind of pods used by the robot
             * goBILDA 4-bar pod is typical
             */
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            /*
             * Set the direction of the X and Y odometry pods
             */
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,  // X pod
                    GoBildaPinpointDriver.EncoderDirection.FORWARD    // Y pod
            );

            /*
             * Recalibrate IMU and reset position
             */
            pinpoint.resetPosAndIMU();
        }
    }
