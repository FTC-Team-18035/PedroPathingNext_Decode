package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclaw;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Drive Witta Vroom Vroom Serqet")
public class DrivewittavroomvroomSerqet extends NextFTCOpMode {
    // Jasper //
        public DrivewittavroomvroomSerqet() {
            addComponents(
                    BulkReadComponent.INSTANCE,
                    BindingsComponent.INSTANCE
            );
        }

        private double power = 0;

        private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
        private final MotorEx frontRightMotor = new MotorEx("front_right");
        private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
        private final MotorEx backRightMotor = new MotorEx("back_right");

        private final MotorEx leftShooter = new MotorEx("left_shooter");
        private final MotorEx rightShooter = new MotorEx("right_shooter").reversed();

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

            telemetry.addData("Shooter Power", power);
        }

        @Override
        public void onUpdate() {
            if (gamepad1.a) {
                power = .25;
            }
            else if (gamepad1.b) {
                power = .5;
            }
            else if (gamepad1.y) {
                power = .75;
            }
            else if (gamepad1.x) {
                power = 1;
            }
            else {
                power = 0;
            }

            rightShooter.setPower(power);
            leftShooter.setPower(power);

            telemetry.update();
        }
    }


