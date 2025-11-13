package org.firstinspires.ftc.teamcode.pedroPathing.paths;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.nextFTC.subsytems.CRservoSubsytemTest;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.nextFTC.subsytems.theclawIntake;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Pedro NextFtc Test")
public class TestAutoNextFtc extends NextFTCOpMode {

    public TestAutoNextFtc() {
        addComponents(
                new SubsystemComponent(CRservoSubsytemTest.INSTANCE, theclawIntake.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Timer pathTimer, opModeTimer, actionTimer;
    private int pathState;


    private final Pose startPos = new Pose(8.6, 85, Math.toRadians(0));
    private final Pose endPos = new Pose(8.6, 140, Math.toRadians(0));

    private Path deliverSample3;



    public void buildPaths() {

        deliverSample3 = new Path(new BezierLine(startPos, endPos));
        deliverSample3.setLinearHeadingInterpolation(startPos.getHeading(), endPos.getHeading());

    }


    public void autonomousUpdatePaths() {
        switch (pathState) {
            case 0:
                new FollowPath(deliverSample3);
                new ParallelGroup(
                        CRservoSubsytemTest.INSTANCE.go,
                        theclawIntake.INSTANCE.close
                );
                telemetry.addData("Position", getComponents());
                new Delay(10);
                actionTimer.resetTimer();
                setPathState(1);
                break;

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void onUpdate() {
        follower().update();
        autonomousUpdatePaths();


        telemetry.update();
    }

    @Override
    public void onInit() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        actionTimer = new Timer();
        opModeTimer.resetTimer();

        buildPaths();
    }


    @Override
    public void onStartButtonPressed() {
        opModeTimer.resetTimer();
        setPathState(0);
    }

}
