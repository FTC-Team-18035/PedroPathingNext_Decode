package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.autonomous;

/*  THIS IS A VERSION OF Serqet_Auto_FarFIX1.java WITH THE FOLLOWING CHANGES:
* Cleaned up unused code
* added comments to the state machine
* added delay timer to the main loop - default 0
* additional pose coordinates for near shooting and 3rd line pickup - no case implementation
* RED POSES are embedded and commented out ready to create RED version
*/

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@Autonomous(name = "BLUE Auto Far Setup", group = "Examples")
public class BLUE_Auto_Far extends OpMode {

    // Optional start delay so you can avoid alliance partners.
    private static final double START_DELAY_SECONDS = 0; //TODO: add delay if need to wait for alliance partner to complete tasks

    private static final double SHOOT_SECONDS = 4.5;           // TODO: Change this if isn't enough time or too much...6 was too much

    private static final double MAX_DRIVE_SPEED = .5; // Change this for the max speed
    private static final double MAX_INTAKE_SPEED = .3; // Change this if we need to intake slower
    private static final double DRIVE_POWER = 0.7;
    private static final double DRIVE_TIMEOUT_SECONDS = 20.0;

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

    private DcMotorEx intake;

    private final int alignStallCounter = 0;
    private final double bestHeadingErrorDeg = Double.MAX_VALUE;

    private Double smoothedDistanceCm = null;
    private double targetDistanceCm = DEFAULT_DISTANCE_CM;

    private BezierPoint holdPoint = null;
    private double holdHeadingRad = 0.0;
    private boolean holdInitialized = false;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13, Math.toRadians(108)); // y(11.1) Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseNear = new Pose(72,72, Math.toRadians(135));  // alternate scoring Pose at field center
    private final Pose pickup1Pose = new Pose(14, 36, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(14.5,60, Math.toRadians(180));
    private final Pose Pickup3Pose = new Pose(14.5,84, Math.toRadians(180));
    private final Pose lineup1Pose = new Pose(41, 36, Math.toRadians(180));
    private final Pose lineup2Pose = new Pose(41.9,60,Math.toRadians(180));
    private final Pose lineup3Pose = new Pose(41.9,84,Math.toRadians(180));
    private final Pose empty = new Pose(16.2,69.8,Math.toRadians(180));
    private final Pose endPose = new Pose(28, 69.7, Math.toRadians(0));

    /*
    * RED POSES
    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose scorePose = new Pose(90.4, 13, Math.toRadians(72));
    private final Pose scorePoseNear = new Pose(72, 72, Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(130.4, 36, Math.toRadians(0)); // 35.2 ?? Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(129.5, 60, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(129.5, 84, Math.toRadians(0));
    private final Pose lineup1Pose = new Pose(102, 36, Math.toRadians(0));  // 34.8
    private final Pose lineup2Pose = new Pose(102.1, 60, Math.toRadians(0));
    private final Pose lineup3Pose = new Pose(102.1, 84, Math.toRadians(0));
    private final Pose empty = new Pose(127.8, 69.8, Math.toRadians(0));
    private final Pose endPose = new Pose(116, 69.7, Math.toRadians(180));
    *
    *  */


    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path pickup1Path;
    private Path pickup2Path;
    private Path emptyPath;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading(), .5);

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading(), .8);

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading(), .5);

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading(), .8);

        score1Path = new Path(new BezierLine(scorePose, lineup2Pose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading(), .5);

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading(), .8);

        pickup2Path = new Path(new BezierLine(pickup2Pose, scorePose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading(), .8);

        score2Path = new Path(new BezierLine(scorePose, empty));
        score2Path.setLinearHeadingInterpolation(scorePose.getHeading(), empty.getHeading(), .8);

        emptyPath = new Path(new BezierLine(empty, endPose));
        emptyPath.setLinearHeadingInterpolation(empty.getHeading(), endPose.getHeading(), .8);


    }

        public void autonomousPathUpdate() {
            /* These are options to check status of current state and move on to avoid stalling
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
            switch (pathState) {
                case 0:     // This moves from start position to far scoring position
                    follower.followPath(scorePreload);
                    setPathState(1);
                    break;

                case 1:     // This shoots the preloaded artifacts
                    if(!follower.isBusy()) {    // Wait until follower movement is finalized to shoot
                        shootForTime(SHOOT_SECONDS);    // begin shooting sequence
                        actionTimer.resetTimer();       // set timer to compare when shooting should be completed
                        setPathState(2);                // advance to next state
                    }
                    break;

                case 2:     // This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                    if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > SHOOT_SECONDS + 1.5) {  // gives the shooter an extra 1.5 seconds to complete
                        follower.followPath(readyPath);     // start movement after shooting completed
                        setPathState(3);                    // advance to next state
                    }
                    break;

                case 3:     // This case waits for alignment and then intakes first row of artifacts
                    if (!follower.isBusy()) {    // wait for path to complete before intake path
                        intake.setPower(1);                 // run intake at start of path
                        shooter.setFeedPower(-1);
                        follower.followPath(lineup1Path);   // begin intake path
                        setPathState(4);                    // advance to next state
                    }
                    break;

                case 4:     // This case waits for intake to complete and then moves to score
                    if (!follower.isBusy()) {       // wait for intake to complete before moving to score
                        intake.setPower(0);         // stop intake
                        shooter.setFeedPower(0);
                        follower.followPath(pickup1Path);   // complete pickup and move to score
                        setPathState(5);            // advance to next state
                    }
                    break;

                case 5:     //  This case shoots and advances to align for second row
                    if(!follower.isBusy()) {            // wait to be in scoring position
                        shootForTime(SHOOT_SECONDS);    // begin shooting sequence
                        actionTimer.resetTimer();       // set timer to compare when shooting should be completed
                        setPathState(6);                // advance to next state
                    }
                    break;

                case 6:     // This is currently the LAST CASE
                    if (!follower.isBusy() && actionTimer.getElapsedTimeSeconds() > SHOOT_SECONDS + 1.5) {
                // This is commented out to cut the opmode short at this stage !!!!
                    /*
                    follower.setMaxPower(MAX_DRIVE_SPEED);
                    follower.followPath(score1Path);
                    setPathState(7) */
                        setPathState(-1);
                    }
                    break;

                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(lineup2Path);
                        setPathState(8);
                    }
                    break;

                case 8:
                    if (!follower.isBusy()) {
                        intake.setPower(1);
                        shooter.setFeedPower(-1);
                        //follower.setMaxPower(MAX_INTAKE_SPEED);
                        follower.followPath(pickup2Path);
                        setPathState(9);
                    }
                    break;

                case 9:
                    if (!follower.isBusy()) {
                        intake.setPower(0);
                        shooter.setFeedPower(0);
                        follower.setMaxPower(MAX_DRIVE_SPEED);
                        follower.followPath(score2Path);
                        setPathState(10);
                    }
                    break;

                case 10:
                    if (!follower.isBusy()) {
                        if(shootForTime(SHOOT_SECONDS) >= SHOOT_SECONDS) {
                            follower.followPath(emptyPath);
                            setPathState(-1);
                        }
                    }
                    break;

                case -1:   //  This case will call a STOP to the opMode
                    if (!follower.isBusy()) {
                        requestOpModeStop();
                    }

            }

        }


        /**
         * These change the states of the paths and actions. It will also reset the timers of the individual switches
         **/
        public void setPathState(int pState) {
            pathState = pState;
            pathTimer.resetTimer();
        }

        /**
         * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
         **/
        @Override
        public void loop() {
            // Delay timer - default is 0 - changed in variable declaration section
            if (START_DELAY_SECONDS < opmodeTimer.getElapsedTimeSeconds()) {
                // These loop the movements of the robot, these must be called continuously in order to work
                follower.update();
                autonomousPathUpdate();
            }

            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }

        /**
         * This method is called once at the init of the OpMode.
         **/
        @Override
        public void init() {
            pathTimer = new Timer();
            opmodeTimer = new Timer();
            actionTimer = new Timer();
            actionTimer.resetTimer();
            opmodeTimer.resetTimer();

            follower = Constants.createFollower(hardwareMap);
            follower.setMaxPower(MAX_DRIVE_SPEED);
            shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(6); // matches BLUEMainTeleOpWORKING
            limelight.start();

            buildPaths();
            follower.setStartingPose(startPose);

        }

        /**
         * This method is called continuously after Init while waiting for "play".
         **/
        @Override
        public void init_loop() {
        }

        /**
         * This method is called once at the start of the OpMode.
        **/
        @Override
        public void start() {
            opmodeTimer.resetTimer();
            actionTimer.resetTimer();
            setPathState(0);
        }

        /**
         * We do not use this because everything should automatically disable
         **/
        @Override
        public void stop() {
        }

        private double shootForTime(double seconds) {
            ElapsedTime timer = new ElapsedTime();

            while (opmodeTimer.getElapsedTimeSeconds() < 30 && timer.seconds() < seconds + 1) {
                if(timer.seconds() > seconds) {
                    stopShoot();
                }
                else {
                    follower.update();
                    updateHold();

                    updateDistanceAndShooterTarget();

                    shooter.setFeedPower(-1.0); // matches BLUEMainTeleOpWORKING feeding direction
                    shooter.update();

                    telemetry.addData("Shooting (s)", timer.seconds());
                    telemetry.update();
                }
            }
            return timer.seconds();
        }

        private void stopShoot() {
            shooter.setTarget(0, .205);
            shooter.update();
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

            // follower.holdPoint(holdPoint, turn);
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


