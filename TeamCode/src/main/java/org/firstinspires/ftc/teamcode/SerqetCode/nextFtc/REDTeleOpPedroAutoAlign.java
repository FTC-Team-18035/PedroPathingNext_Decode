package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@TeleOp//(name = "RED Main TeleOp *TEST*", group = "PedroPathing")
public class REDTeleOpPedroAutoAlign extends LinearOpMode {

    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS
       ---------------------------------------------------------
       Used to compute distance to target from vertical angle.
       Units are meters for heights, degrees for angles.
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;

    /* =========================================================
       ALIGNMENT CONTROL CONSTANTS
       ---------------------------------------------------------
       These control how aggressively and how precisely the robot
       attempts to rotate to face the AprilTag.
       ========================================================= */


    private double offsetHeading;
    // Proportional gain converting heading error → turn power
    private static final double ALIGN_KP = -0.015;

    // Minimum turn command to overcome drivetrain friction
    private static final double ALIGN_MIN_CMD = 0.09;

    // "Good enough" heading accuracy (degrees)
    private static final double ALIGN_ACCEPTABLE_ERROR = 0.35;

    // Smallest improvement considered meaningful (degrees)
    private static final double ALIGN_MIN_IMPROVEMENT = 0.02;

    // How many loops with no improvement we allow before declaring a stall
    private static final int ALIGN_STALL_CYCLES = 8;

    // Counts consecutive loops without improvement
    private int alignStallCounter = 0;
    private static MecanumConstants mecanumConstants;
    private static PinpointConstants pinpointConstants;

    /* =========================================================
       SHOOTER SAFETIES & FILTERING
       ========================================================= */

    // Allowed error between target and actual flywheel velocity
    private static final double FLYWHEEL_TOLERANCE = 50;

    // If the tag disappears for too long, abort the shot
    private static final long TAG_TIMEOUT_MS = 500;

    // Exponential smoothing factor for distance calculation
    private static final double ALPHA = 0.3;
    private static double turn;
    private static BezierPoint point;

    // Telemetry values
    public double leftError;
    public double rightError;

    /* =========================================================
       HARDWARE
       ========================================================= */
    private Limelight3A limelight;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;
    private DcMotorEx intake, lift;

    public LLResult result;

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private enum ShootState {
        IDLE,        // Driver control
        ALIGNING,    // Vision-based heading refinement
        SPINNING_UP, // Flywheel + hood positioning
        FEEDING      // Launch the note
    }

    private ShootState shootState = ShootState.IDLE;

    /* =========================================================
       ALIGNMENT TRACKING
       ========================================================= */

    // Best heading error seen so far during this alignment attempt
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    /* =========================================================
       DISTANCE TRACKING
       ========================================================= */
    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = 0.0;

    /* =========================================================
       TAG VISIBILITY TRACKING
       ========================================================= */
    private long lastTagSeenTimeMs = 0;

    private double heading = 0;

    /* =========================================================
       AUTO ALIGN (one-shot tx)
       ========================================================= */
    private boolean alignStarted = false;
    private Pose holdPose;
    private Path alignPath;

    private static double wrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
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
        mecanumConstants = new MecanumConstants()
                .leftFrontMotorName("front_left")
                .leftRearMotorName("back_left")
                .rightFrontMotorName("front_right")
                .rightRearMotorName("back_right");

        pinpointConstants = new PinpointConstants()
                .hardwareMapName("pinpoint");

        follower = new FollowerBuilder(new FollowerConstants(), hardwareMap)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();

        waitForStart();
        follower.startTeleOpDrive();

        /* ---------------- Main Loop ---------------- */
        while (opModeIsActive()) {

            handleIntake();
            handleShootingStateMachine();
            handleLift();
            handleDrive();

            shooter.update();
            follower.update();
            telemetry.update();

            /*if(shootState != ShootState.IDLE) {
                follower.holdPoint(point, follower.getHeading());
            }
            else {
                follower.breakFollowing();
                handleDrive();
            }*/
        }

        limelight.stop();
    }

    /* =========================================================
       DRIVER CONTROL
       ========================================================= */
    private void handleDrive() {
        // Disable manual driving during shooting sequence
        if (shootState != ShootState.IDLE) return;

        double scalar = gamepad1.left_trigger > 0.5 ? 1.0 : 0.5;

        if(gamepad1.left_trigger > .75 && gamepad1.right_trigger > .75) {
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
            abortShot();
            return;
        }

        /* -------- Start shooting sequence -------- */
        if (gamepad1.a && shootState == ShootState.IDLE) {
            point = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());
            bestHeadingErrorDeg = Double.MAX_VALUE;
            smoothedDistanceCm = null;
            shootState = ShootState.ALIGNING;

            // We'll grab tx once in ALIGNING and start a turn path that holds the endpoint.
            alignStarted = false;
            holdPose = null;
            alignPath = null;
        }

        // Always update Limelight result once per loop so other states can safely use it.
        result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid()
                && !result.getFiducialResults().isEmpty();

        if (tagValid) {
            lastTagSeenTimeMs = System.currentTimeMillis();
        }

        /* -------- Safety: lost tag -------- */
        if (shootState != ShootState.IDLE &&
                System.currentTimeMillis() - lastTagSeenTimeMs > TAG_TIMEOUT_MS) {
            abortShot();
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

                // Poll tx ONCE, then start a short path that ends with hold=true.
                if (!alignStarted) {
                    double tx = result.getTx() + 2; // your existing offset

                    double startHeadingDeg = follower.getHeading();
                    offsetHeading = wrapDeg(startHeadingDeg + tx);

                    // Hold position is the pose at button press, just with an updated heading.
                    holdPose = new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            offsetHeading
                    );

                    // Make a tiny "turn" path (very small translation) so Pedro engages path follower,
                    // and then holds the end pose to fight defense.
                    Pose startPose = new Pose(
                            follower.getPose().getX(),
                            follower.getPose().getY(),
                            follower.getPose().getHeading()
                    );
                    Pose endPose = new Pose(
                            follower.getPose().getX() + 0.5,
                            follower.getPose().getY(),
                            offsetHeading
                    );

                    alignPath = new Path(new BezierLine(startPose, endPose));
                    alignPath.setConstantHeadingInterpolation(offsetHeading);
                    follower.followPath(alignPath, true);
                    alignStarted = true;
                }

                // Once the follower finishes, it will hold the end pose (because hold=true).
                if (!follower.isBusy()) {
                    shootState = ShootState.SPINNING_UP;
                }

                break;
            }

            /* =====================================================
               SPINNING UP
               ===================================================== */
            case SPINNING_UP: {
        // DON'T overwrite Pedro's hold-at-end with teleop commands.
        // We just let the follower continue holding the end of the ALIGNING path.
        if (!tagValid) break;

                double distanceMeters =
                        (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(Math.toRadians(result.getTy()
                                        + LIMELIGHT_MOUNT_ANGLE));

                double distanceCm = distanceMeters * 100.0;

                smoothedDistanceCm = smoothedDistanceCm == null
                        ? distanceCm
                        : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

                targetDistanceCm = smoothedDistanceCm;

                double targetVelocity =
                        TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
                double targetAngle =
                        TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);

                shooter.setTarget(targetVelocity, targetAngle);

                leftError = Math.abs(Math.abs(
                        shooter.getLeftVelocity()) - targetVelocity);
                rightError = Math.abs(Math.abs(
                        shooter.getRightVelocity()) - targetVelocity);

                if (leftError < FLYWHEEL_TOLERANCE ||
                        rightError < FLYWHEEL_TOLERANCE) {
                    shootState = ShootState.FEEDING;
                }

                break;
            }

            /* =====================================================
               FEEDING
               ===================================================== */
            case FEEDING:
                // Continue holding end pose autonomously
                shooter.setFeedPower(-1.0);
                break;

            default:
                break;
        }

        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Flywheel Error",
                (leftError + rightError) / 2.0);
        telemetry.addData("Best Align Error (deg)",
                bestHeadingErrorDeg);
        telemetry.addData("Distance (cm)", targetDistanceCm);
        telemetry.addData("Pos", point);
        telemetry.addData("Angle", turn);
    }

    /* =========================================================
       ABORT
       ========================================================= */
    private void abortShot() {
        shootState = ShootState.IDLE;
        shooter.stop();
        shooter.setFeedPower(0.0);
        smoothedDistanceCm = null;

        alignStarted = false;
        holdPose = null;
        alignPath = null;

        // Restore driver controls
        follower.startTeleOpDrive();
    }

    private void handleLift() {
        if(gamepad1.dpad_up && gamepad1.left_trigger > .75) {       //TODO Changed it so you have to be holding the left trigger to use the lift
            lift.setTargetPosition(3600);
            lift.setPower(1);
        }
    }
}
