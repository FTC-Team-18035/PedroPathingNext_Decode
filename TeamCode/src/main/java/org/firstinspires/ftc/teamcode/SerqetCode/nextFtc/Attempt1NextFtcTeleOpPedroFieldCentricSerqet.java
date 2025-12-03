package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclaw;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Field Centric Pedro Serqet")
public class Attempt1NextFtcTeleOpPedroFieldCentricSerqet extends NextFTCOpMode {
    public Attempt1NextFtcTeleOpPedroFieldCentricSerqet() {
        addComponents(
             new SubsystemComponent(/*CRservoSubsytemTest.INSTANCE*/ theclaw.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private String telemetryValue = null;

    private final MotorEx leftShooter = new MotorEx("left_shooter");
    private final MotorEx rightShooter = new MotorEx("right_shooter");

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
        if (gamepad1.a) {
            leftShooter.setPower(1);
            rightShooter.setPower(1);
        }
        else {
            leftShooter.setPower(0);
            rightShooter.setPower(0);
        }
    }
}
