package org.firstinspires.ftc.teamcode.SerqetCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.TrajectorySCRIMMAGE;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.ShooterSubsystemSCRIMMAGE;


@Autonomous(name = "Shoot and Scoot Far", preselectTeleOp = "MainTeleOp")
public class TImeBasedShootAndScootSCRIMMAGE extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private ShooterSubsystemSCRIMMAGE shooter;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new ShooterSubsystemSCRIMMAGE(hardwareMap);
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double shooterVelocity = TrajectorySCRIMMAGE.CalculateVelocity(345.12);
        double shooterAngle = TrajectorySCRIMMAGE.CalculateAngle(345.12);

        waitForStart();
        timer.reset();

        while (timer.seconds() < 7) {
            shooter.setTarget(shooterVelocity, shooterAngle);
            shooter.setFeedPower(-1);
            shooter.update();
        }

        shooter.stop();
        shooter.setFeedPower(0);

        frontLeft.setPower(.5);
        frontRight.setPower(.5);
        backLeft.setPower(.5);
        backRight.setPower(.5);

        sleep(500);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

}
