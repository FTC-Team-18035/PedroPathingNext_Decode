package org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclaw;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;

@Disabled
@TeleOp(name = "Field Centric Pedro")
public class Attempt1NextFtcTeleOpPedroFieldCentric extends NextFTCOpMode {
    public Attempt1NextFtcTeleOpPedroFieldCentric() {
        addComponents(
             new SubsystemComponent(theclaw.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    public DriverControlledCommand driverControlled;
    private String telemetryValue = null;

    @Override
    public void onStartButtonPressed() {
        telemetry.addData("", telemetryValue);

        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                false
        );
        driverControlled.schedule();

    }

    @Override
    public void onUpdate() {
        if (gamepad1.right_trigger >= .75) {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX().negate(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    false
            );
        }
        else {
            driverControlled = new PedroDriverControlled(
                    Gamepads.gamepad1().leftStickY().negate(),
                    Gamepads.gamepad1().leftStickX().negate(),
                    Gamepads.gamepad1().rightStickX().negate(),
                    true
            );
        }
        driverControlled.schedule();
    }
}
