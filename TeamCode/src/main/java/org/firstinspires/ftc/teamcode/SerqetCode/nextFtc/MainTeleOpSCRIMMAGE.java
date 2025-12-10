package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;



import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;

@TeleOp(name = "MainTeleOp for Scrimmage", group = "PedroPathing")
public class MainTeleOpSCRIMMAGE extends LinearOpMode {
    private static final double SHOOTER_TARGET_VELOCITY = 500.0; // ticks per second
    private static final double SHOOTER_TARGET_SERVO = 0.205; // within the clamped safe range
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_POWER = 1.0;
    private Follower follower;
    private ShooterSubsystemSCRIMMAGE shooter;
    private DcMotorEx intake;

    double targetDistance;
    private Servo vault;

    @Override
    public void runOpMode() {
        targetDistance = 345.12;
        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);
        //vault = hardwareMap.get(Servo.class, "vault_release");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        waitForStart();
        follower.startTeleOpDrive();
        while (opModeIsActive()) {
            drive();
            handleIntakeAndFeed();
            handleShooter();
            shooter.update();
            follower.update();
            pushTelemetry();
        }
    }

    private void drive() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, false, 0.0);
    }

    private void handleIntakeAndFeed() {
        double intakePower = 0.0;
        double feedPower = 0.0;
        if (gamepad1.left_bumper) {
            intakePower = INTAKE_POWER;
            feedPower = -FEED_POWER;
            //vault.setPosition(1);
        } else if (gamepad1.right_bumper) {
            intakePower = -INTAKE_POWER;
            feedPower = FEED_POWER;
        }
        else {
            //vault.setPosition(0);
        }
        intake.setPower(intakePower);
        shooter.setFeedPower(feedPower);
    }

    private void handleShooter() {
        if (gamepad1.a) {
            //--------------------------------
            //shooter.Calculate(210);
            double Velocity = TrajectorySCRIMMAGE.CalculateVelocity(targetDistance);
            double Angle = TrajectorySCRIMMAGE.CalculateAngle(targetDistance);
            shooter.setTarget(Velocity, Angle);
         //   shooter.setTarget(SHOOTER_TARGET_VELOCITY, SHOOTER_TARGET_SERVO);
            shooter.setFeedPower(-1);
        } else {
            shooter.stop();
        }
    }

    private void pushTelemetry() {
        telemetry.addData("Shooter target", TrajectorySCRIMMAGE.CalculateVelocity(targetDistance));
        telemetry.addData("Left vel", shooter.getLeftVelocity());
        telemetry.addData("Right vel", shooter.getRightVelocity());
        telemetry.addData("Servo pos", shooter.getServoPosition());
        telemetry.addData("launchAngle",TrajectorySCRIMMAGE.CalculateAngle(targetDistance));
        telemetry.update();
    }
}