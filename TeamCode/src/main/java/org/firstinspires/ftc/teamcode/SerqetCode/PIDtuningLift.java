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


@Configurable
@TeleOp
public class PIDtuningLift extends OpMode {

    private PIDController LiftController;

    public static double lp = 0,
            li = 0,
            ld = 0;

    public static double lf = 0;

    public static int liftTarget = 0;

    private final double lift_ticks_in_degrees = 1.068055;   // TODO - set for specific motor

    private DcMotorEx Lift;


    @Override
    public void init(){
        LiftController = new PIDController(lp, li, ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Lift = hardwareMap.get(DcMotorEx.class, "lift");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop(){
        LiftController.setPID(lp, li, ld);


        int liftPos = Lift.getCurrentPosition();

        double lPID = LiftController.calculate(liftPos, liftTarget);


        double liftFF = Math.cos(Math.toRadians(liftTarget / lift_ticks_in_degrees)) * lf;

        double liftPower = lPID + liftFF;

        Lift.setPower(liftPower);

        telemetry.addData("Lift pos ", liftPos);
        telemetry.addData("lift target ", liftTarget);
        telemetry.update();

    }

}
