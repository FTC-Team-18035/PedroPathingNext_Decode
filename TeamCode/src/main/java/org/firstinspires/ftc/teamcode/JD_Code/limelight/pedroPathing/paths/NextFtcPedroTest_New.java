package org.firstinspires.ftc.teamcode.JD_Code.limelight.pedroPathing.paths;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.CRservoSubsytemTest;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclaw;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclawIntake;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@Autonomous(name = "Pedro NextFtc Test")
public class NextFtcPedroTest_New extends NextFTCOpMode {
   public NextFtcPedroTest_New() {
       addComponents(
               new SubsystemComponent(theclaw.INSTANCE, theclawIntake.INSTANCE, CRservoSubsytemTest.INSTANCE),
               new PedroComponent(Constants::createFollower)
       );
   }

   private Timer pathTimer, actionTimer, opModeTimer;

   private int pathState;

   private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
   private final Pose driveForwards = new Pose(30, 0, Math.toRadians(0));
   private final Pose driveBackwards = new Pose(0, 0, Math.toRadians(0));

   public Path driveForward, driveBackward;

    public void buildPaths() {
        driveForward = new Path(new BezierLine(startPose, driveForwards));
        driveForward.setLinearHeadingInterpolation(startPose.getHeading(), driveForwards.getHeading());

        driveBackward = new Path(new BezierLine(driveForwards, driveBackwards));
        driveBackward.setLinearHeadingInterpolation(driveForwards.getHeading(), driveBackwards.getHeading());
    }

    public Command autonomousPathUpdate() {
            return new SequentialGroup(
                new FollowPath(driveForward),

                theclawIntake.INSTANCE.open,

                new FollowPath(driveBackward)
                );
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void onUpdate() {
        PedroComponent.follower().update();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", PedroComponent.follower().getPose().getX());
        telemetry.addData("y", PedroComponent.follower().getPose().getY());
        telemetry.addData("heading", PedroComponent.follower().getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        buildPaths();
        PedroComponent.follower().setStartingPose(startPose);
    }

    @Override
    public void onStartButtonPressed() {
        autonomousPathUpdate().schedule();
        opModeTimer.resetTimer();
        setPathState(0);
    }
}

