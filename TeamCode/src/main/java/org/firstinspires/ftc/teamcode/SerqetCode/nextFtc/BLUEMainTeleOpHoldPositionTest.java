package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;
@Disabled
@TeleOp(name = "BLUE Main TeleOp", group = "PedroPathing")
public class BLUEMainTeleOpHoldPositionTest extends LinearOpMode {
    /* =========================================================
       LIMELIGHT GEOMETRY CONSTANTS
       ========================================================= */
    private static final double LIMELIGHT_HEIGHT = 0.24;
    private static final double TARGET_HEIGHT = 0.75;
    private static final double LIMELIGHT_MOUNT_ANGLE = 13.0;
    /* =========================================================
       ALIGNMENT CONTROL CONSTANTS
       ========================================================= */
    private static final double ALIGN_KP = -0.015;
    private static final double ALIGN_MIN_CMD = 0.09;
    private static final double ALIGN_ACCEPTABLE_ERROR = 0.35;
    private static final double ALIGN_MIN_IMPROVEMENT = 0.02;
    private static final int ALIGN_STALL_CYCLES = 8;
    private int alignStallCounter = 0;
    private double bestHeadingErrorDeg = Double.MAX_VALUE;
    /* =========================================================
       SHOOTER SAFETIES & FILTERING
       ========================================================= */
    private static final double FLYWHEEL_TOLERANCE = 50;
    private static final long TAG_TIMEOUT_MS = 500;
    private static final double ALPHA = 0.3;
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
       HEADING REFERENCE (your existing feature)
       ========================================================= */
    private double heading = 0;
    /* =========================================================
       HOLDPOINT CONTROL
       ========================================================= */
    private boolean holdActive = false;
    private Pose holdPose = null;
    @Override
    public void runOpMode() {
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
        limelight.pipelineSwitch(6);
        limelight.start();
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
        follower.setMaxPower(1);
        while (opModeIsActive()) {
            handleShootingStateMachine(); // may engage holdPoint + override drive
            handleDrive();                // only runs in IDLE
            handleIntake();               // only runs in IDLE
            handleLift();
            shooter.update();
            follower.update();
            telemetry.addData("Shoot State", shootState);
            telemetry.addData("HoldActive", holdActive);
            telemetry.addData("HoldPose",
                    holdPose == null ? "null" :
                            String.format("(%.2f, %.2f, %.2f)",
                                    holdPose.getX(), holdPose.getY(), holdPose.getHeading()));
            telemetry.addData("Flywheel Error", (leftError + rightError) / 2.0);
            telemetry.addData("Best Align Error (deg)", bestHeadingErrorDeg);
            telemetry.addData("Distance (cm)", targetDistanceCm);
            telemetry.update();
        }
        limelight.stop();
    }
    /* =========================================================
       DRIVER CONTROL (IDLE ONLY)
       ========================================================= */
    private void handleDrive() {
        if (shootState != ShootState.IDLE) return;
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
       INTAKE CONTROL (IDLE ONLY)
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
        // Release A => STOP holding and return driver control immediately
        if (!gamepad1.a && shootState != ShootState.IDLE) {
            abortToDriver();
            return;
        }
        // Press A to start
        if (gamepad1.a && shootState == ShootState.IDLE) {
            shootState = ShootState.ALIGNING;
            bestHeadingErrorDeg = Double.MAX_VALUE;
            alignStallCounter = 0;
            smoothedDistanceCm = null;
            // reset hold
            holdActive = false;
            holdPose = null;
        }
        if (shootState == ShootState.IDLE) return;
        LLResult result = limelight.getLatestResult();
        boolean tagValid = result != null && result.isValid()
                && !result.getFiducialResults().isEmpty();
        if (tagValid) {
            lastTagSeenTimeMs = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - lastTagSeenTimeMs > TAG_TIMEOUT_MS) {
            abortToDriver();
            return;
        }
        switch (shootState) {
            case ALIGNING: {
                if (gamepad1.b) { // manual override
                    shootState = ShootState.SPINNING_UP;
                    break;
                }
                double tx = result.getTx() + 2;
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
                // IMPORTANT: during ALIGNING we are intentionally overriding driver drive
                follower.setTeleOpDrive(0, 0, turn, false, heading);
                if (absError <= ALIGN_ACCEPTABLE_ERROR &&
                        alignStallCounter >= ALIGN_STALL_CYCLES) {
                    shootState = ShootState.SPINNING_UP;
                }
                break;
            }
            case SPINNING_UP: {
                // IMPORTANT: do NOT call setTeleOpDrive every loop if you want holdPoint later,
                // BUT here we haven't engaged holdPoint yet. Still, keeping this at zero is fine.
                follower.setTeleOpDrive(0, 0, 0, false, heading);
                double distanceMeters =
                        (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(Math.toRadians(result.getTy() + LIMELIGHT_MOUNT_ANGLE));
                double distanceCm = distanceMeters * 100.0;
                smoothedDistanceCm = (smoothedDistanceCm == null)
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
                    // Engage hold on entry to FEEDING
                    engageHoldPointOnce();
                }
                break;
            }
            case FEEDING: {
                // CRITICAL: while holding, DO NOT call setTeleOpDrive().
                // Let the follower's holdPoint controller own the drivetrain.
                if (!holdActive) {
                    engageHoldPointOnce();
                }
                shooter.setFeedPower(-1.0);
                break;
            }
        }
    }
    /* =========================================================
       PEDRO HOLDPOINT ENGAGE (ONCE)
       ========================================================= */
    private void engageHoldPointOnce() {
        Pose p = follower.getPose();
        holdPose = new Pose(p.getX(), p.getY(), follower.getHeading());
        // ---- YOU MAY NEED TO EDIT THIS LINE to match your exact signature ----
        // Common patterns in newer builds are either:
        //   follower.holdPoint(holdPose);
        // or follower.holdPoint(holdPose.getX(), holdPose.getY(), holdPose.getHeading());
        follower.holdPoint(holdPose);
        // ---------------------------------------------------------------------
        holdActive = true;
    }
    /* =========================================================
       ABORT => return driver control immediately
       ========================================================= */
    private void abortToDriver() {
        shootState = ShootState.IDLE;
        holdActive = false;
        holdPose = null;
        smoothedDistanceCm = null;
        shooter.stop();
        shooter.setFeedPower(0.0);
        // IMPORTANT: switch follower back to driver mode explicitly
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
}