package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierPoint;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@TeleOp
public class REDMainTeleOp_TESTHoldPoint extends LinearOpMode {

    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;

    /* =========================================================
       SHOOTER SAFETIES & FILTERING
       ========================================================= */
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final long TAG_TIMEOUT_MS = 500;
    private static final double ALPHA = 0.3;

    /* =========================================================
       LIMELIGHT OFFSETS 
       ========================================================= */
    private static final double LIMELIGHT_TX_OFFSET_DEG = 2.0;

    /* =========================================================
       HOLDPOINT HEADING CONTROLLER 
       ========================================================= */
    private static final double HOLD_HEADING_KP = 0.45;
    private static final double HOLD_MAX_TURN = 0.45;
    private static final double HOLD_MIN_TURN_CMD = 0.06;
    private static final double HOLD_HEADING_DEADBAND_RAD = Math.toRadians(1.0);

    /* =========================================================
       TELEMETRY
       ========================================================= */
    public double leftError;
    public double rightError;

    /* =========================================================
       HARDWARE
       ========================================================= */
    private Limelight3A limelight;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;
    private DcMotorEx intake, lift;

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private enum ShootState {
        IDLE,
        ALIGNING,
        SPINNING_UP,
        FEEDING
    }

    private ShootState shootState = ShootState.IDLE;

    /* =========================================================
       DISTANCE TRACKING
       ========================================================= */
    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = 0.0;

    /* =========================================================
       TAG VISIBILITY TRACKING
       ========================================================= */
    private long lastTagSeenTimeMs = 0;

    /* =========================================================
       HEADING REFERENCE 
       ========================================================= */
    private double heading = 0;

    /* =========================================================
       HOLDPOINT TARGETS
       ========================================================= */
    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;

    // We only read tx ONCE in ALIGNING and convert it to a new heading setpoint.
    private boolean alignTxCaptured = false;

    @Override
    public void runOpMode() {

        /* ---------------- Hardware Initialization ---------------- */
        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // AprilTag pipeline
        limelight.start();

        /* ---------------- Drive / Localization Setup ---------------- */
        MecanumConstants mecanumConstants = new MecanumConstants()
                .leftFrontMotorName("front_left")
                .leftRearMotorName("back_left")
                .rightFrontMotorName("front_right")
                .rightRearMotorName("back_right");

        PinpointConstants pinpointConstants = new PinpointConstants()
                .hardwareMapName("pinpoint");

        follower = new FollowerBuilder(new FollowerConstants(), hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();

        waitForStart();
        follower.startTeleOpDrive();

        /* ---------------- Main Loop ---------------- */
        while (opModeIsActive()) {
            handleShootingStateMachine();
            handleDrive();
            handleIntake();
            handleLift();

            shooter.update();
            follower.update();

            telemetry.addData("Shoot State", shootState);
            telemetry.addData("HoldHeading(deg)", Math.toDegrees(holdHeadingRad));
            telemetry.addData("Distance (cm)", targetDistanceCm);
            telemetry.addData("Flywheel Err", (leftError + rightError) / 2.0);
            telemetry.update();
        }

        limelight.stop();
    }

    /* =========================================================
       DRIVER CONTROL
       ========================================================= */
    private void handleDrive() {
        // If we’re not idle, drivetrain ownership belongs to holdPoint().
        if (shootState != ShootState.IDLE) {
            updateHoldPoint();
            return;
        }

        double scalar = gamepad1.left_trigger > 0.5 ? 1.0 : 0.5;

        if (gamepad1.left_trigger > .75 && gamepad1.right_trigger > .75) {
            heading = follower.getHeading();
        }

        follower.setTeleOpDrive(
                scalar * gamepad1.left_stick_y,
                scalar * gamepad1.left_stick_x,
                scalar * gamepad1.right_stick_x,
                false,
                heading
        );
    }

    /* =========================================================
       INTAKE CONTROL
       ========================================================= */
    private void handleIntake() {
        if (shootState != ShootState.IDLE) return;

        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
            shooter.setFeedPower(-1.0);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
            shooter.setFeedPower(1.0);
        } else {
            intake.setPower(0.0);
            shooter.setFeedPower(0.0);
        }
    }

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private void handleShootingStateMachine() {

        /* -------- Abort if trigger released -------- */
        if (!gamepad1.a && shootState != ShootState.IDLE) {
            abortToDriver();
            return;
        }

        /* -------- Start shooting sequence -------- */
        if (gamepad1.a && shootState == ShootState.IDLE) {
            shootState = ShootState.ALIGNING;

            // Initialize hold targets from current pose
            holdPoint = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());
            holdHeadingRad = follower.getPose().getHeading();
            alignTxCaptured = false;

            smoothedDistanceCm = null;
        }

        if (shootState == ShootState.IDLE) return;

        LLResult result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid() && !result.getFiducialResults().isEmpty();

        if (tagValid) {
            lastTagSeenTimeMs = System.currentTimeMillis();
        }

        /* -------- Safety: lost tag -------- */
        if (System.currentTimeMillis() - lastTagSeenTimeMs > TAG_TIMEOUT_MS) {
            abortToDriver();
            return;
        }

        switch (shootState) {

            case ALIGNING: {
                if (!tagValid) break;

                // Manual override → skip alignment and keep scoring
                if (gamepad1.b) {
                    shootState = ShootState.SPINNING_UP;
                    break;
                }

                // Poll tx ONCE, offset it, and convert to a new heading setpoint.
                if (!alignTxCaptured) {
                    double txDeg = result.getTx() + LIMELIGHT_TX_OFFSET_DEG;
                    holdHeadingRad = follower.getPose().getHeading() + Math.toRadians(txDeg);
                    alignTxCaptured = true;
                }

                // Drivetrain command is done via holdPoint() inside handleDrive().
                // Transition once we’re inside the deadband.
                double headingError = angleWrapRad(holdHeadingRad - follower.getPose().getHeading());
                if (Math.abs(headingError) < HOLD_HEADING_DEADBAND_RAD) {
                    shootState = ShootState.SPINNING_UP;
                }

                break;
            }

            case SPINNING_UP: {
                if (!tagValid) break;

                double distanceMeters =
                        (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(Math.toRadians(result.getTy() + LIMELIGHT_MOUNT_ANGLE));

                double distanceCm = distanceMeters * 100.0;

                smoothedDistanceCm = smoothedDistanceCm == null
                        ? distanceCm
                        : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

                targetDistanceCm = smoothedDistanceCm;

                double targetVelocity = TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
                double targetAngle = TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);

                shooter.setTarget(targetVelocity, targetAngle);

                leftError = Math.abs(Math.abs(shooter.getLeftVelocity()) - targetVelocity);
                rightError = Math.abs(Math.abs(shooter.getRightVelocity()) - targetVelocity);

                if (leftError < FLYWHEEL_TOLERANCE || rightError < FLYWHEEL_TOLERANCE) {
                    shootState = ShootState.FEEDING;
                }

                break;
            }

            case FEEDING: {
                shooter.setFeedPower(-1.0);
                break;
            }
        }
    }

    /* =========================================================
       PEDRO HOLDPOINT UPDATE
       ========================================================= */
    private void updateHoldPoint() {
        if (holdPoint == null) {
            holdPoint = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());
        }

        // Pedro API in this repo expects the second argument to be the heading (rad) to hold,
        // not a turn power command.
        follower.holdPoint(holdPoint, holdHeadingRad);
    }

    /* =========================================================
       ABORT
       ========================================================= */
    private void abortToDriver() {
        shootState = ShootState.IDLE;
        smoothedDistanceCm = null;
        shooter.stop();
        shooter.setFeedPower(0.0);

        alignTxCaptured = false;
        holdPoint = null;
        holdHeadingRad = 0.0;

        follower.startTeleOpDrive();
    }

    /* =========================================================
       LIFT
       ========================================================= */
    private void handleLift() {
        if (gamepad1.dpad_up && gamepad1.left_trigger > .75) {
            lift.setTargetPosition(3600);
            lift.setPower(1);
        }
    }

    /* =========================================================
       SMALL MATH HELPERS
       ========================================================= */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double angleWrapRad(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
