package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@Disabled
@Autonomous(name = "Shooter Auto Test")
public class Pedro_ShooterTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private double shooterVelocity;
    private double shooterAngle;

    private ShooterSubsystemSCRIMMAGE shooter;

    private final Pose startPose = new Pose(18, 118, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose endPose = new Pose(18, 118, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    private Path scorePreload;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, endPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.setTarget(shooterVelocity, shooterAngle);
                shooter.setFeedPower(1);
                setPathState(1);
            case 1:
                if(actionTimer.getElapsedTimeSeconds() >= 10) {
                    follower.followPath(scorePreload);
                    setPathState(-1);
                }
            case -1:
                if(!follower.isBusy()) {
                    requestOpModeStop();
                }
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        shooter.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);
        shooterVelocity = TrajectorySCRIMMAGE.CalculateVelocity(345.12);
        shooterAngle = TrajectorySCRIMMAGE.CalculateAngle(345.12);


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        actionTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}