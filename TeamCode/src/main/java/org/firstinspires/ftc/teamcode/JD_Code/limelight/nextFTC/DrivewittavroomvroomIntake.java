package org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclawIntake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
@Disabled
@TeleOp(name = "Drive Witta Vroom Vroom Intake")
public class DrivewittavroomvroomIntake extends NextFTCOpMode {
    // Jasper //
        public DrivewittavroomvroomIntake() {
            addComponents(
                    new SubsystemComponent(theclawIntake.INSTANCE),
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

            Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                    .whenBecomesTrue(
                            theclawIntake.INSTANCE.close
                    );
            Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                    theclawIntake.INSTANCE.open
            );
        }
    }


