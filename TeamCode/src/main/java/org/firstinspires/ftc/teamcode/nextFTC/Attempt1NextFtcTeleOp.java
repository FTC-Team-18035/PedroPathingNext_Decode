package org.firstinspires.ftc.teamcode.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC TeleOp Program Java")
public class Attempt1NextFtcTeleOp extends NextFTCOpMode {
    public Attempt1NextFtcTeleOp() {
        addComponents(
             //   new SubsystemComponent(LiftSubSystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("Front Left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("Front Right");
    private final MotorEx backLeftMotor = new MotorEx("Back Left").reversed();
    private final MotorEx backRightMotor = new MotorEx("Back Right");

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

       /* Gamepads.gamepad2().dpadUp()
                .whenBecomesTrue(
                        LiftSubSystem.INSTANCE.toHigh
                );

        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(
                        LiftSubSystem.INSTANCE.toMiddle
                );

        Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                        LiftSubSystem.INSTANCE.toLow
                );*/
    }
}
