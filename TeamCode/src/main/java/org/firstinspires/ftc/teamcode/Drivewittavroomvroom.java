package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Drive Witta Vroom Vroom")
public class TeleOpProgram extends NextFTCOpMode {
        public TeleOpProgram() {
            addComponents(
                    new SubsystemComponent(Claw.INSTANCE),
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
                    Gamepads.gamepad1().rightStickY()
            );
            driverControlled.schedule();

            Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                    .whenBecomesTrue(
                            Claw.INSTANCE.close
                    );
            Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                    Claw.INSTANCE.open
            );
        }
    }


