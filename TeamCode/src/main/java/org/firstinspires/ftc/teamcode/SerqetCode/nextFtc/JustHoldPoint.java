package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;

@Disabled
@Configurable
@TeleOp
public class JustHoldPoint extends OpMode {
    public boolean holdInitialized;
    public boolean holdStart;
    public double holdHeadingRad;
    public static final double HOLD_HEADING_KP = 0.45;
    public static final double HOLD_MAX_TURN = 0.45;
    public static final double HOLD_MIN_TURN_CMD = 0.06;
    public static final double HOLD_HEADING_DEADBAND_RAD = Math.toRadians(1.0);
    public BezierPoint holdPoint = new BezierPoint(0, 0);
    private Follower follower;
    private Pose  endPose, middlePose, startPose;
    //private BezierPoint startPose;
    private PathChain holdPath;
    private Path drive;
    private double turn = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        holdInitialized = false;
    }

    @Override
    public void start(){
        follower.update();
        //beginHoldFromCurrentPose();
    }
    @Override
    public void loop() {
        if(gamepad1.a) {
            holdStart = false;
            follower.setTeleOpDrive(
                    0,
                    0,
                    0,
                    false,
                    0
            );
        }
        else if(gamepad1.yWasPressed()) {
            holdInitialized = false;
            updateHold();
            follower.update();
            holdStart = true;
        }
        if(holdStart){
            updateHold();
            follower.update();
        }
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
        turn = HOLD_HEADING_KP * headingError;

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