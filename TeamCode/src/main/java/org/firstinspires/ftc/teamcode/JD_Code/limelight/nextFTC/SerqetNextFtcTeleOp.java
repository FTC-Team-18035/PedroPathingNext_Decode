package org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled
@TeleOp(name = "Serqet NextFTC TeleOp")
public class SerqetNextFtcTeleOp extends NextFTCOpMode {
    public SerqetNextFtcTeleOp() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private String telemetryValue = null;

    // Names for DECODE season robot SERQET
    private final MotorEx frontLeft = new MotorEx("front_left").reversed();
    private final MotorEx frontRight = new MotorEx("front_right");
    private final MotorEx backLeft = new MotorEx("back_left").reversed();
    private final MotorEx backRight = new MotorEx("back_right");

    // private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()    // needed for Pedro field centric

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("", telemetryValue);
        // enable CHASSIS
        // TODO - determine motor directions for proper operation
        Command driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
                // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
        );
        driverControlled.schedule();

      /*  Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                        LiftSubSystem.INSTANCE.toHigh
                );*/
    }
}
