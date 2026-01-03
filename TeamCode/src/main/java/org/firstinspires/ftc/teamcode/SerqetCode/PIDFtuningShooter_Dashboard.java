package org.firstinspires.ftc.teamcode.SerqetCode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp (name = "Shooter Tuner Dashboard")
public class PIDFtuningShooter_Dashboard extends OpMode {
    public PIDFtuningShooter_Dashboard() { }

    private PIDFController ShooterController;
    public static double sp = 0, si = 0, sd = 0, sf = 0;
    public static double shooterTarget = 0;
    private final double shooter_ticks_in_degrees = 28;   // set this for specific motors completed 12/6
    private DcMotorEx leftShooter;
    private DcMotorEx rightShooter;
    public static TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        ShooterController = new PIDFController(sp, si, sd, sf);

        leftShooter = hardwareMap.get(DcMotorEx.class, "left_shooter");
        rightShooter = hardwareMap.get(DcMotorEx.class, "right_shooter");

        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);  // reverse one motor

        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    public void loop(){
        ShooterController.setPIDF(sp, si, sd, sf);          // use new PIDF values

        double shooterVel = leftShooter.getVelocity();      // read velocity

        double sPID = ShooterController.calculate(shooterVel, shooterTarget);   // calculate path to new velocity target

        double shooterFF = Math.cos(Math.toRadians(shooterTarget / shooter_ticks_in_degrees)) * sf;  // more math

        double shooterPower = sPID + shooterFF;             // calculate corrected power value

        double error = shooterTarget - shooterVel;

        leftShooter.setPower(shooterPower);                 // set power to motors
        rightShooter.setPower(-shooterPower);               // TODO - verify motor directions !!!

        telemetry.addData("Shooter Velocity", shooterVel);      // output to Panels
        telemetry.addData("Shooter Target ",shooterTarget);
        telemetry.addData("error ", error);
        telemetry.update();

    }

}
