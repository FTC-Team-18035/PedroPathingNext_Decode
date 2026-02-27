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

@TeleOp(name="HoldPositionTestBLUE")
public class BLUEMainTeleOpHoldPositionTest extends LinearOpMode {

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

    /* =========================================================
       SHOOTER SAFETIES & FILTERING
       ========================================================= */

    // Allowed error between target and actual flywheel velocity
    private static final double FLYWHEEL_TOLERANCE = 50;

    // If the tag disappears for too long, abort the shot
    private static final long TAG_TIMEOUT_MS = 500;

    // Exponential smoothing factor for distance calculation
    private static final double ALPHA = 0.3;

    // Telemetry values
    public double leftError;
    public double rightError;

    /* =========================================================
       HARDWARE
       ========================================================= */
    private Limelight3A limelight;  // Limelight declaration
    private Follower follower;      // Instance of the Pedro follower
    private ShooterSubsystemSCRIMMAGE shooter;  // Instance of our shooter class
    private DcMotorEx intake, lift;     // Motor declarations

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private enum ShootState {
        IDLE,        // Driver control
        ALIGNING,    // Vision-based heading refinement
        SPINNING_UP, // Flywheel + hood positioning
        FEEDING      // Launch the note
    }

    private ShootState shootState = ShootState.IDLE;     // Initial shoot state

    /* =========================================================
       ALIGNMENT TRACKING
       ========================================================= */

    // Best heading error seen so far during this alignment attempt
    private double bestHeadingErrorDeg = Double.MAX_VALUE;

    /* =========================================================
       DISTANCE TRACKING
       ========================================================= */
    private Double smoothedDistanceCm = null;   // Smoothed distance from target
    private double targetDistanceCm = 0.0;      // Distance from target

    /* =========================================================
       TAG VISIBILITY TRACKING
       ========================================================= */
    private long lastTagSeenTimeMs = 0;     // How long it's been since the tag has been read
    private double heading = 0;     // Robot's heading

    public double scalar;       // Scalar for our turbo mode that switches it to full power as opposed to half

    public BezierPoint holdPoint;   // Instance of the point we want to hold

    public boolean holdInitalized = false;  // Hold has begun

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
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(6); // AprilTag pipeline for Blue
        limelight.start();

        /* ---------------- Drive / Localization Setup ---------------- */
        MecanumConstants mecanumConstants = new MecanumConstants()      // Pedro drive control initialization
                .leftFrontMotorName("front_left")
                .leftRearMotorName("back_left")
                .rightFrontMotorName("front_right")
                .rightRearMotorName("back_right");

        PinpointConstants pinpointConstants = new PinpointConstants()   // Pedro Pinpoint constants initialization
                .hardwareMapName("pinpoint");

        follower = new FollowerBuilder(new FollowerConstants(), hardwareMap)    // Pedro follower initialization
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();

        waitForStart();
        follower.startTeleOpDrive();    // Give driver control

        /* ---------------- Main Loop ---------------- */
        while (opModeIsActive()) {
            handleDrive();  // Handles drive inputs
            handleIntake(); // Handles Intake inputs
            handleShootingStateMachine();   // Updates the shooter states
            handleLift();   // Handles the lift inputs

            shooter.update();   // Updates the shooter every frame
            follower.update();  // Updates the follower every frame
            telemetry.update(); // Updates Telemetry every frame
        }

        limelight.stop();   // Stops limelight from streaming
    }

    /* =========================================================
       DRIVER CONTROL
       ========================================================= */
    private void handleDrive() {
        // Disable manual driving during shooting sequence
        if (shootState != ShootState.IDLE) return;

        if (gamepad1.left_trigger > .5){    // Activates turbo mode
            scalar = 1;
        }
        else if (gamepad1.left_trigger < .5) {  // Resets back to default if turbo is not needed
            scalar = .5;
        }

        if(gamepad1.left_trigger > .75 && gamepad1.right_trigger > .75) {   // Resets field centric
            heading = follower.getHeading();    // Resets our heading variable
        }
        follower.setTeleOpDrive(    // Does all the logic for driving
                scalar * gamepad1.left_stick_y,     // The scalar variable is set to what speed we want
                scalar * gamepad1.left_stick_x,
                scalar * gamepad1.right_stick_x,
                false,
                heading     // Uses the heading variable every frame to set the heading
        );
    }

    /* =========================================================
       INTAKE CONTROL
       ========================================================= */
    private void handleIntake() {
        if (shootState != ShootState.IDLE) return;

        if (gamepad1.left_bumper) {     // Checks if the left bumper was pressed
            intake.setPower(1.0);   // Sets intake power to full
            shooter.setFeedPower(-1.0); // Activates the vault system to pull artifacts towards the shooter
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
            shooter.setFeedPower(1.0);
        } else {
            intake.setPower(0.0);   // If neither bumpers are pressed it stops the intake and feeder
            shooter.setFeedPower(0.0);
        }
    }

    /* =========================================================
       SHOOTING STATE MACHINE
       ========================================================= */
    private void handleShootingStateMachine() {

        /* -------- Abort if trigger released -------- */
        if (!gamepad1.a && shootState != ShootState.IDLE) {
            abortShot();            // Aborts the shot if we release the shoot button
            holdInitalized = false; // Cancels Hold
            return;
        }

        /* -------- Start shooting sequence -------- */
        if (gamepad1.a && shootState == ShootState.IDLE) {
            holdInitalized = false;     // Sets hold to false since we aren't aligned yet
            shootState = ShootState.ALIGNING;   // Begin aligning
            bestHeadingErrorDeg = Double.MAX_VALUE; // Error margin
            alignStallCounter = 0;
            smoothedDistanceCm = null;
        }

        LLResult result = limelight.getLatestResult();  // Gets the results from limelight
        boolean tagValid = result != null && result.isValid() // Saves if it's a valid tag or not
                && !result.getFiducialResults().isEmpty();

        if (tagValid) { // Checks if it is a valid tag
            lastTagSeenTimeMs = System.currentTimeMillis(); // Updates the last time a tag was seen
        }

        /* -------- Safety: lost tag -------- */
        if (shootState != ShootState.IDLE &&
                System.currentTimeMillis() - lastTagSeenTimeMs > TAG_TIMEOUT_MS) { // Checks if the tag was seen too long ago
            abortShot();    // If it was it aborts the shot
            holdInitalized = false; // And cancels hold
            return;
        }

        switch (shootState) {

            case ALIGNING: {    // The aligning state
                holdPoint = new BezierPoint(follower.getPose().getX(), follower.getPose().getY());  // This is where we set the point we want to hold
                heading = follower.getHeading() + Math.toRadians(limelight.getLatestResult().getTx());  // aligns to the goal by setting the heading to the target x
            }
            if(!holdInitalized) {   // Checks if hold has been initialized yet
                follower.holdPoint(holdPoint, heading); // If not it calls hold point using our point we created along with our heading
                holdInitalized = true;  // Initializes the hold
            }
            else{
                break;
            }

            /* =====================================================
               SPINNING UP
               ===================================================== */
            case SPINNING_UP: { // Flywheels begin spinning up
                follower.update();  // Updates the follower to hold the point
                double distanceMeters =     // Gets the meters away from the target tag
                        (TARGET_HEIGHT - LIMELIGHT_HEIGHT) /
                                Math.tan(Math.toRadians(result.getTy()
                                        + LIMELIGHT_MOUNT_ANGLE));

                double distanceCm = distanceMeters * 100.0; // Converts it into centimeters

                smoothedDistanceCm = smoothedDistanceCm == null // Smooths out the cm
                        ? distanceCm
                        : ALPHA * distanceCm + (1 - ALPHA) * smoothedDistanceCm;

                targetDistanceCm = smoothedDistanceCm;  // Sets the target to the smoothed target

                double targetVelocity =     // Passes our target into our calculations class to figure out the velocity needed
                        TrajectorySCRIMMAGE.CalculateVelocity(targetDistanceCm);
                double targetAngle =        // Passes our target into our calculations class to figure out the angle needed
                        TrajectorySCRIMMAGE.CalculateAngle(targetDistanceCm);

                shooter.setTarget(targetVelocity, targetAngle); // Sets the shooter's target to the values returned by the calculation classes

                leftError = Math.abs(Math.abs(  // Gets the error from the left side of the tag
                        shooter.getLeftVelocity()) - targetVelocity);
                rightError = Math.abs(Math.abs(     // Gets the error from the right side of the tag
                        shooter.getRightVelocity()) - targetVelocity);

                if (leftError < FLYWHEEL_TOLERANCE ||   // Checks if the error on either side is too big
                        rightError < FLYWHEEL_TOLERANCE) {
                    shootState = ShootState.FEEDING;    // If the error is not too big then it begins feeding artifacts
                }

                break;
            }

            /* =====================================================
               FEEDING
               ===================================================== */
            case FEEDING:
                follower.update();  // Updates the follower so it continues to hold point
                shooter.setFeedPower(-1.0); // Starts passing through artifacts
                break;

            default:
                break;
        }

        telemetry.addData("Shoot State", shootState);   // Shows us the shoot state
        telemetry.addData("Flywheel Error",     // Shows us the error on either side of the tag
                (leftError + rightError) / 2.0);
        telemetry.addData("Best Align Error (deg)",     // Shows us the smallest error in the heading
                bestHeadingErrorDeg);
        telemetry.addData("Distance (cm)", targetDistanceCm);   // Shows the cm from the goal
    }

    /* =========================================================
       ABORT
       ========================================================= */
    private void abortShot() {  // If the shot is aborted
        shootState = ShootState.IDLE;       // Sets the state to Idle
        shooter.stop();                 // Stops the flywheels
        shooter.setFeedPower(0.0);  // Stops the feeder
        smoothedDistanceCm = null;  // Resets the distance to target
    }

    private void handleLift() {     // Checks if the lift buttons have been pressed and if the position is below the max
        if(gamepad1.dpad_up && gamepad1.left_trigger > .75 && lift.getCurrentPosition() < 3600) {    //TODO Changed it so you have to be holding the left trigger to run the lift
            lift.setPower(1);   // Raises the lift up
        }
        else {
            lift.setPower(0);   // Stops the lift
        }
    }
}