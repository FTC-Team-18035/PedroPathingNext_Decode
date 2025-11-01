package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Lift Test")
public class LiftTest extends LinearOpMode {

    private PIDController liftController;

    public static double lp = 0, li = 0, ld = 0;
    public static double lf = 0;
    public static int targetLift = 0;
    private final double lift_ticks_in_degrees = 0;

    private DcMotorEx lift;

    private final int MAX_TARGET_LIFT = 0;

    private double LiftPower;
    @Override
    public void runOpMode() throws InterruptedException {
        liftController = new PIDController(lp, li, ld);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {

            liftController.setPID(lp, li, ld);

            int liftPos = lift.getCurrentPosition();

            double lpid = liftController.calculate(liftPos, targetLift);

            double liftFF = Math.cos(Math.toRadians(targetLift / lift_ticks_in_degrees)) * lf;

            double liftPower = lpid + liftFF;

            lift.setPower(liftPower);

            telemetry.addData("Lift pos ", liftPos);
            telemetry.addData("Lift target ", targetLift);
            telemetry.update();
        }
    }
}
