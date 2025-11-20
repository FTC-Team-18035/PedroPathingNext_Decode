package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Full Serqet")
public class SerqetMain extends NextFTCOpMode {
    public SerqetMain() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right");
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right");

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


        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(
                        Shooter.INSTANCE.shoot // Error Here
                );


        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(
                        Shooter.INSTANCE.stop // Error Here
                );

        driverControlled.schedule();
    }
}


