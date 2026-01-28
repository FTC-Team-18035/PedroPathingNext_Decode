package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class JustHoldPoint extends OpMode {

    private Follower follower;
    private Pose  endPose, middlePose;
    private BezierPoint startPose;
    private PathChain holdPath;

    @Override
    public void init() {
        startPose = new BezierPoint(new Pose(0, 0, 0));
        middlePose = new Pose(0.5, 0, 0);
        endPose = new Pose(0, 0, 0);
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

      /*  holdPath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, middlePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), middlePose.getHeading())
                .addPath(new BezierLine(middlePose, endPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), endPose.getHeading())
                .build();*/

    }

    @Override
    public void start(){
      //  follower.followPath();
    }
    @Override
    public void loop() {
        follower.update();

    }
}