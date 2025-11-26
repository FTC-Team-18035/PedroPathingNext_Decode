package org.firstinspires.ftc.teamcode.SerqetCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;

@Configurable
@TeleOp
public class PIDtuningShooter extends OpMode {

    private PIDController ShooterController;

    public static double sp = 0,
            si = 0, sd = 0;

    public static double sf = 0;

    public static int shooterTarget = 0;

    private final double shooter_ticks_in_degrees = 0;

    private DcMotorEx LeftShooter;


    @Override
    public void init(){
        ShooterController = new PIDController(sp, si, sd);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LeftShooter = hardwareMap.get(DcMotorEx.class, "left_shooter");

        LeftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop(){
        ShooterController.setPID(sp, si, sd);


        int shooterPos = LeftShooter.getCurrentPosition();

        double sPID = ShooterController.calculate(shooterPos, shooterTarget);


        double shooterFF = Math.cos(Math.toRadians(shooterTarget / shooter_ticks_in_degrees)) * sf;

        double shooterPower = sPID + shooterFF;

        LeftShooter.setPower(shooterPower);

        telemetry.addData("Lift pos ", shooterPos);
        telemetry.addData("lift target ", shooterTarget);
        telemetry.update();

    }

}
