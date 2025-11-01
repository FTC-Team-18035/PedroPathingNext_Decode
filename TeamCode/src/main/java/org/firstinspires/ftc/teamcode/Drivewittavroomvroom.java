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
public class Drivewittavroomvroom extends NextFTCOpMode {
    // Jasper //
        public Drivewittavroomvroom() {
            addComponents(
                    new SubsystemComponent(theclaw.INSTANCE),
                    BulkReadComponent.INSTANCE,
                    BindingsComponent.INSTANCE
            );
        }

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
                    Gamepads.gamepad1().rightStickY()
            );
            driverControlled.schedule();

            Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                    .whenBecomesTrue(
                            theclaw.INSTANCE.close
                    );
            Gamepads.gamepad2().leftBumper().whenBecomesTrue(
                    theclaw.INSTANCE.open
            );
        }
    }


