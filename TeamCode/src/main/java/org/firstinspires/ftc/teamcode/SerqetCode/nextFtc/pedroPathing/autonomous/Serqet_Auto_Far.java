package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.autonomous; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;

@Autonomous(name = "Example Auto", group = "Examples")
 public class Serqet_Auto_Far extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(55.1, 11.1, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13.1, Math.toRadians(112)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(14, 45, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(14.5,58.6, Math.toRadians(180));
    private final Pose lineup1Pose = new Pose(55, 11, Math.toRadians(112));
    private final Pose lineup2Pose = new Pose(41.9,59,Math.toRadians(112));
    private final Pose empty = new Pose(16.2,69.8,Math.toRadians(112));
    private final Pose endPose = new Pose(28, 69.7, Math.toRadians(180));


    private Path scorePreload;
    private Path score1Path;
    private Path score2Path;
    private Path readyPath;
    private Path lineup1Path;
    private Path lineup2Path;
    private Path pickup1Path;
    private Path pickup2Path;
    private Path emptyPath;
    private Path endPath;
    private Path path8;
    private PathChain grabPickup1;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        readyPath = new Path(new BezierLine(scorePose, lineup1Pose));
        readyPath.setLinearHeadingInterpolation(scorePose.getHeading(), lineup1Pose.getHeading());

        lineup1Path = new Path(new BezierLine(lineup1Pose, pickup1Pose)) ;
        lineup1Path.setLinearHeadingInterpolation(lineup1Pose.getHeading(), pickup1Pose.getHeading());

        pickup1Path = new Path(new BezierLine(pickup1Pose, scorePose));
        pickup1Path.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading());

        score1Path = new Path(new BezierLine(scorePose, lineup2Pose ));
        score1Path.setLinearHeadingInterpolation(scorePose.getHeading(), lineup2Pose.getHeading());

        lineup2Path = new Path(new BezierLine(lineup2Pose, pickup2Pose));
        lineup2Path.setLinearHeadingInterpolation(lineup2Pose.getHeading(), pickup2Pose.getHeading());

        pickup2Path = new Path(new BezierLine(pickup2Pose, scorePose));
        pickup2Path.setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading());

        score2Path = new Path(new BezierLine(scorePose, empty));
        score2Path.setLinearHeadingInterpolation(scorePose.getHeading(), empty.getHeading());

        emptyPath = new Path(new BezierLine(empty, endPose));
        emptyPath.setLinearHeadingInterpolation(empty.getHeading(), endPose.getHeading());

        /*path8 = new Path(new BezierLine(pose8, pose8));
        path8.setLinearHeadingInterpolation(pose8.getHeading(), pose8.getHeading());


    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
       /* grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();*/

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
            {
                setPathState(1);
                break;
            }
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                }
                /* Score Preload */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(readyPath);
            {
                setPathState(1_5);
            }
            break;
            case 1_5:
                if (!follower.isBusy()) {
                    follower.followPath(lineup1Path);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(pickup1Path);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(score1Path);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) ;
            {
                follower.followPath(lineup2Path);
                setPathState(5);
            }
            break;
            case 5:
                if (!follower.isBusy()) ;
            {
                follower.followPath(pickup2Path);
                setPathState(6);
            }
            break;
            case 6:
                if (!follower.isBusy()) ;
            {
                follower.followPath(score2Path);
                setPathState(7);
            }
            break;
            case 7:
                if (!follower.isBusy()) ; {
                follower.followPath(emptyPath);
                setPathState(-1);
            }
            break;

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

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

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
     * It runs all the setup actions, including building paths and starting the path system
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
}
