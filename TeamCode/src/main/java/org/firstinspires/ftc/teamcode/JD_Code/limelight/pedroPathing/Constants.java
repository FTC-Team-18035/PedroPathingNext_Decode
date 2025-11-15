package org.firstinspires.ftc.teamcode.JD_Code.limelight.pedroPathing;

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
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

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
            .leftFrontMotorName("Front Left")
            .leftRearMotorName("Back Left")
            .rightFrontMotorName("Front Right")
            .rightRearMotorName("Back Right")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(54.8677)
            .yVelocity(38.0781);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(.001978956)
            .strafeTicksToInches(.001978956)
            .forwardPodY(-3.623)
            .strafePodX(3.770)
            .forwardEncoder_HardwareMapName("Front Right")
            .strafeEncoder_HardwareMapName("Back Right")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
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
