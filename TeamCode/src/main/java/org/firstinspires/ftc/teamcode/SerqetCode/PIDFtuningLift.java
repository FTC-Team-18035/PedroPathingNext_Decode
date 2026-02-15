package org.firstinspires.ftc.teamcode.SerqetCode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Configurable
@TeleOp
public class PIDFtuningLift extends OpMode {

    private PIDFController LiftController;

    public static double lp = 0, li = 0, ld = 0, lf = 0;

    public static int liftTarget = 0;

    private final double lift_ticks_in_degrees = 1.068055;   // TODO - set for specific motor

    private DcMotorEx Lift;

    private static TelemetryManager panelsTelemetry;

    @Override
    public void init(){
        LiftController = new PIDFController(lp, li, ld, lf);

        Lift = hardwareMap.get(DcMotorEx.class, "lift");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    @Override
    public void loop(){

        LiftController.setPIDF(lp, li, ld, lf);

        int liftPos = Lift.getCurrentPosition();

        double lPID = LiftController.calculate(liftPos, liftTarget);

        double liftFF = Math.cos(Math.toRadians(liftTarget / lift_ticks_in_degrees)) * lf;

        double liftPower = lPID + liftFF;

        Lift.setPower(liftPower);

        double error = liftTarget - liftPos;

        panelsTelemetry.debug("Lift pos " + liftPos);
        panelsTelemetry.debug("lift target " + liftTarget);
        panelsTelemetry.debug( "error " + error + "close enough for gov't work!");
        panelsTelemetry.update(telemetry);
    }

}
