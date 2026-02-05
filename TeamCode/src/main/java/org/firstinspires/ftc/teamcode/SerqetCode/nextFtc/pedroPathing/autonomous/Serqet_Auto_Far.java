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

    private final Pose startPose = new Pose(55.1, 11.1, 90); // Start Pose of our robot.
    private final Pose scorePose = new Pose(53.6, 13.1, 112); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pose1 = new Pose(55, 11, 112);
    private final Pose pose2 = new Pose(14,45, 180);
    private final Pose pose3 = new Pose(53.6, 13.1, 180);
    private final Pose pose4 = new Pose(41.9,59,112);
    private final Pose pose5 = new Pose(14.5,58.6,180);
    private final Pose pose6 = new Pose(53.5,13.4,180);
    private final Pose pose7 = new Pose(16.2,69.8,112);
    private final Pose pose8 = new Pose(28, 69.7, 180);


    private Path scorePreload;
    private Path path1;
    private Path path1_5;
    private Path path2;
    private Path path3;
    private Path path4;
    private Path path5;
    private Path path6;
    private Path path7;
    private Path path8;
    private PathChain grabPickup1;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        path1 = new Path(new BezierLine(scorePose, pose1));
        path1.setLinearHeadingInterpolation(scorePose.getHeading(), pose1.getHeading());

        path1_5 = new Path(new BezierLine(pose1, pose2)) ;
        path1_5.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        path2 = new Path(new BezierLine(pose2, scorePose));
        path2.setLinearHeadingInterpolation(pose2.getHeading(), scorePose.getHeading());

        path3 = new Path(new BezierLine(scorePose, pose4 ));
        path3.setLinearHeadingInterpolation(scorePose.getHeading(), pose4.getHeading());

        path4 = new Path(new BezierLine(pose4, pose5));
        path4.setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading());

        path5 = new Path(new BezierLine(pose5, scorePose));
        path5.setLinearHeadingInterpolation(pose5.getHeading(), scorePose.getHeading());

        path6 = new Path(new BezierLine(scorePose, pose7));
        path6.setLinearHeadingInterpolation(scorePose.getHeading(), pose7.getHeading());

        path7 = new Path(new BezierLine(pose7, pose8));
        path7.setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading());

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
                follower.followPath(path1);
            {
                setPathState(1_5);
            }
            break;
            case 1_5:
                if (!follower.isBusy()) {
                    follower.followPath(path1_5);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path2);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) ;
            {
                follower.followPath(path4);
                setPathState(5);
            }
            break;
            case 5:
                if (!follower.isBusy()) ;
            {
                follower.followPath(path5);
                setPathState(6);
            }
            break;
            case 6:
                if (!follower.isBusy()) ;
            {
                follower.followPath(path6);
                setPathState(7);
            }
            break;
            case 7:
                if (!follower.isBusy()) ; {
                follower.followPath(path7);
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
