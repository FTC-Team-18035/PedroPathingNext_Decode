package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Lift Test")
public class LiftTest extends LinearOpMode {

    private PIDController LiftController;

    public static double lp = 0, li = 0, ld = 0;
    public static double lf = 0;
    public static int targetLift = 0;
    private final double lift_ticks_in_degrees = 0;

    private DcMotorEx Lift;

    private final int MAX_TARGET_LIFT = 0;

    private double LiftPower;
    @Override
    public void runOpMode() throws InterruptedException {
        LiftController = new PIDController(lp, li, ld);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Lift = hardwareMap.get(DcMotorEx.class, "lift");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();

        while(opModeIsActive()) {

            LiftController.setPID(lp, li, ld);

            int LiftPos = Lift.getCurrentPosition();

            double Lpid = LiftController.calculate(LiftPos, targetLift);

            double LiftFF = Math.cos(Math.toRadians(targetLift / lift_ticks_in_degrees)) * lf;

            double LiftPower = Lpid + LiftFF;

            Lift.setPower(LiftPower);

            telemetry.addData("Lift pos ", LiftPos);
            telemetry.addData("lift target ", targetLift);
            telemetry.update();
        }
    }
}
