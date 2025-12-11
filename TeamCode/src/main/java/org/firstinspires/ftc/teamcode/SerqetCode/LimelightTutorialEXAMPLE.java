package org.firstinspires.ftc.teamcode.SerqetCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class LimelightTutorialEXAMPLE extends OpMode {

    private Limelight3A limelight;

    private double robotDistance;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.initialize();
    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        Pose2D orientation = pinpoint.getPosition();
        limelight.updateRobotOrientation(orientation.getHeading(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            robotDistance = getDistanceFromTage(llResult.getTa());
            telemetry.addData("distance", robotDistance);
            telemetry.addData("tx", llResult.getTx());
            telemetry.addData("ty", llResult.getTy());
            telemetry.addData("ta", llResult.getTa());
        }

        telemetry.update();

    }

    public double getDistanceFromTage(double ta) {
        double scale = 0; // TODO input scale from measuring distances
        double distance = (scale / ta);

        return distance;
    }
}
