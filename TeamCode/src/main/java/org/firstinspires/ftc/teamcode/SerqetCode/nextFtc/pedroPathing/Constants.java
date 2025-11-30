package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(16.60148)
            .forwardZeroPowerAcceleration(-32.3427)
            .lateralZeroPowerAcceleration(-67.989)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.001)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.31, 0, 0, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(3, 0, 0, 0))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.005, 0, 0.00027, 0, 0)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("front_left")
            .leftRearMotorName("back_left")
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(54.8677)
            .yVelocity(38.0781);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(.001978956)
            .strafeTicksToInches(.001978956)
            .forwardPodY(-3.623)
            .strafePodX(3.770)
            .forwardEncoder_HardwareMapName("front_left") // TODO - get wiring config
            .strafeEncoder_HardwareMapName("back_left")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("pinpoint")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )
            );

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            500,
            1,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
