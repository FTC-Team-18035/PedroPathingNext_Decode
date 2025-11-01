package org.firstinspires.ftc.teamcode.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Autonomous(name = "NextFTC Autonomous Program Java")
public class Attempt1NextFtcAutonomous extends NextFTCOpMode {
    public Attempt1NextFtcAutonomous() {
        addComponents(
             //   new SubsystemComponent(LiftSubSystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("Front Left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("Front Right");
    private final MotorEx backLeftMotor = new MotorEx("Back Left").reversed();
    private final MotorEx backRightMotor = new MotorEx("Back Right");

  //  follower().breakFollowing();
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
