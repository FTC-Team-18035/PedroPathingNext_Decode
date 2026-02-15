package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@Autonomous(name = "BLUE Auto: Align+Shoot+Drive 12\"", group = "Serqet", preselectTeleOp = "BLUE Main TeleOp")
public class BLUE_AutoAlignShootDrive12 extends LinearOpMode {
    private DcMotorEx frontRight, frontLeft, backRight, backLeft;

    /* =========================================================
       TUNABLE AUTONOMOUS CONSTANTS
       ========================================================= */
    private static final double SHOOT_SECONDS = 4.5;           // TODO: Change this if isn't enough time or too much...6 was too much
    private static final double DRIVE_FORWARD_INCHES = 20.0; //TODO: Change if distance is wrong

    private static final double DRIVE_POWER = 0.7;
    private static final double DRIVE_TIMEOUT_SECONDS = 20.0;

    // Optional start delay so you can avoid alliance partners.
    private static final double START_DELAY_SECONDS = 0; //TODO: add delay if need to wait for alliance partner to complete tasks

    // If robot drives the wrong direction, flip this between -1 and +1.
    private static final double FORWARD_SIGN = 1.0;

    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS (matches TeleOp)
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;

    /* =========================================================
       ALIGNMENT CONTROL CONSTANTS (matches TeleOp)
       ========================================================= */
    private static final double ALIGN_KP = -0.015;
    private static final double ALIGN_MIN_CMD = 0.09;
    private static final double ALIGN_ACCEPTABLE_ERROR = 0.35;
    private static final double ALIGN_MIN_IMPROVEMENT = 0.02;
    private static final int ALIGN_STALL_CYCLES = 8;
    private static final double ALIGN_TIMEOUT_SECONDS = 3.0;

    // Blue-side horizontal offset used in BLUEMainTeleOpWORKING
    private static final double BLUE_TX_OFFSET_DEG = -2.0;

    /* =========================================================
       SHOOTER FILTERING / SAFETIES
       ========================================================= */
    private static final double ALPHA = 0.3;
    private static final double DEFAULT_DISTANCE_CM = 260.0; // fallback if tag isn't visible

    /* =========================================================
       HOLD (to keep robot stable while shooting)
       ========================================================= */
    private static final double HOLD_HEADING_KP = 1.8;
    private static final double HOLD_MAX_TURN = 0.45;
    private static final double HOLD_MIN_TURN_CMD = 0.06;
    private static final double HOLD_HEADING_DEADBAND_RAD = Math.toRadians(1.0);

    private Limelight3A limelight;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;

    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;

    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6); // matches BLUEMainTeleOpWORKING
        limelight.start();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addLine("BLUE Auto ready: align -> shoot -> drive 12\"");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            limelight.stop();
            return;
        }

        follower.startTeleOpDrive();

        runStartDelay(START_DELAY_SECONDS);

        //alignToAprilTag();
        //beginHoldFromCurrentPose();

        shootForTime(SHOOT_SECONDS);
        stopShooting();


        driveForwardInches(DRIVE_FORWARD_INCHES); //TODO: This method holds position!! Don't know why. TeleOp?
        stopDrive();

        limelight.stop();
    }

    private void runStartDelay(double seconds) {
        if (seconds <= 0.0) return;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            double remaining = Math.max(0.0, seconds - timer.seconds());

            // Keep drivetrain stopped during delay.
            follower.setTeleOpDrive(0, 0, 0, false);
            follower.update();

            telemetry.addData("Start delay (s)", remaining);
            telemetry.update();
            idle();
        }
    }

    private void alignToAprilTag() {
        alignStallCounter = 0;
        bestHeadingErrorDeg = Double.MAX_VALUE;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < ALIGN_TIMEOUT_SECONDS) {
            follower.update();

            LLResult result = limelight.getLatestResult();
            boolean tagValid = result != null && result.isValid()
                    && result.getFiducialResults() != null
                    && !result.getFiducialResults().isEmpty();

            if (!tagValid) {
                follower.setTeleOpDrive(0, 0, 0, false);
                telemetry.addLine("Align: no tag yet");
                telemetry.update();
                continue;
            }

            double tx = result.getTx() + BLUE_TX_OFFSET_DEG;
            double absError = Math.abs(tx);

            if (absError < bestHeadingErrorDeg - ALIGN_MIN_IMPROVEMENT) {
                bestHeadingErrorDeg = absError;
                alignStallCounter = 0;
            } else {
                alignStallCounter++;
            }

            double turn = ALIGN_KP * (-tx);

            if (Math.abs(turn) < ALIGN_MIN_CMD && absError > ALIGN_ACCEPTABLE_ERROR) {
                turn = Math.copySign(ALIGN_MIN_CMD, turn);
            }

            follower.setTeleOpDrive(0, 0, turn, false);

            telemetry.addData("Align tx", tx);
            telemetry.addData("Align abs", absError);
            telemetry.addData("Align stall", alignStallCounter);
            telemetry.update();

            if (absError <= ALIGN_ACCEPTABLE_ERROR && alignStallCounter >= ALIGN_STALL_CYCLES) {
                follower.setTeleOpDrive(0, 0, 0, false);
                return;
            }
        }

        // timeout: stop turning
        follower.setTeleOpDrive(0, 0, 0, false);
    }

    private void shootForTime(double seconds) {
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            follower.update();
            updateHold();

            updateDistanceAndShooterTarget();

            shooter.setFeedPower(-1.0); // matches BLUEMainTeleOpWORKING feeding direction
            shooter.update();

            telemetry.addData("Shooting (s)", timer.seconds());
            telemetry.addData("Distance (cm)", targetDistanceCm);
            telemetry.update();
        }
    }

    private void updateDistanceAndShooterTarget() {
        LLResult result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid()
                && result.getFiducialResults() != null
                && !result.getFiducialResults().isEmpty();

        double distanceCm = targetDistanceCm;

        if (tagValid) {
            double distanceMeters = (TARGET_HEIGHT - LIMELIGHT_HEIGHT)
                    / Math.tan(Math.toRadians(result.getTy() + LIMELIGHT_MOUNT_ANGLE));
            distanceCm = distanceMeters * 100.0;
        }

        smoothedDistanceCm = smoothedDistanceCm == null
                ? distanceCm
                : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

        targetDistanceCm = smoothedDistanceCm;

        double targetVelocity = TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
        double targetAngle = TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);
        shooter.setTarget(targetVelocity, targetAngle);
    }

    private void stopShooting() {
        shooter.setFeedPower(0.0);
        shooter.stop();
    }

    //TODO: for some reason, this method holds position nearly perfectly, find out why.
    /*private void driveForwardInches(double inches) {
        Pose startPose = follower.getPose();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {

            Pose currentPose = follower.getPose();
            double dx = currentPose.getX() - startPose.getX();
            double dy = currentPose.getY() - startPose.getY();
            double traveled = Math.hypot(dx, dy);

            if (traveled >= inches) {
                break;
            }

            follower.setTeleOpDrive(-.75, 0, 0, false);
            telemetry.addData("Drive traveled (in)", traveled);
            telemetry.update();
        }
    }
     */
    private void driveForwardInches(double inches) {
        Pose startPose = follower.getPose();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_SECONDS) {

            follower.update();

            Pose currentPose = follower.getPose();
            double dx = currentPose.getX() - startPose.getX();
            double dy = currentPose.getY() - startPose.getY();
            double traveled = Math.hypot(dx, dy);

            if (traveled >= inches) {
                frontRight.setPower(0);
                frontLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
                break;
            }

            frontRight.setPower(.6);
            frontLeft.setPower(.6);
            backRight.setPower(.6);
            backLeft.setPower(.6);

            telemetry.addData("Drive traveled (in)", traveled);
            telemetry.update();
        }
    }

    private void stopDrive() {
        follower.setTeleOpDrive(0, 0, 0, false);
        follower.update();
    }

    private void beginHoldFromCurrentPose() {
        if (holdInitialized) return;
        holdPoint = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());
        holdHeadingRad = follower.getPose().getHeading();
        holdInitialized = true;
    }

    private void updateHold() {
        if (!holdInitialized) {
            beginHoldFromCurrentPose();
        }

        double headingError = angleWrapRad(holdHeadingRad - follower.getPose().getHeading());
        double turn = HOLD_HEADING_KP * headingError;

        if (Math.abs(headingError) < HOLD_HEADING_DEADBAND_RAD) {
            turn = 0.0;
        } else {
            if (Math.abs(turn) < HOLD_MIN_TURN_CMD) {
                turn = Math.copySign(HOLD_MIN_TURN_CMD, turn);
            }
            turn = clamp(turn, -HOLD_MAX_TURN, HOLD_MAX_TURN);
        }

        follower.holdPoint(holdPoint, turn);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double angleWrapRad(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
