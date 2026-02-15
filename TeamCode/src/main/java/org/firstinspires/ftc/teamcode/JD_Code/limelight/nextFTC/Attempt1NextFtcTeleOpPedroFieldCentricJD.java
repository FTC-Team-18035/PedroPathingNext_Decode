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
import dev.nextftc.hardware.impl.MotorEx;
@Disabled
@TeleOp(name = "Field Centric Pedro Johnny")
public class Attempt1NextFtcTeleOpPedroFieldCentricJD extends NextFTCOpMode {
    public Attempt1NextFtcTeleOpPedroFieldCentricJD() {
        addComponents(
             new SubsystemComponent(/*CRservoSubsytemTest.INSTANCE*/ theclaw.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private String telemetryValue = null;

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("Front Left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("Front Right");
    private final MotorEx backLeftMotor = new MotorEx("Back Left").reversed();
    private final MotorEx backRightMotor = new MotorEx("Back Right");

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

      /* Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(
                        CRservoSubsytemTest.INSTANCE.stop
                );

        Gamepads.gamepad2().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(
                        CRservoSubsytemTest.INSTANCE.go

                );
        Gamepads.gamepad1().rightTrigger().greaterThan(0.2)
                .whenBecomesTrue(
                        theclaw.INSTANCE.close
                );
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(
                theclaw.INSTANCE.open
        );
      /*  Gamepads.gamepad1().x()
                .whenBecomesTrue(
                        theclawIntake.INSTANCE.close
                );
        Gamepads.gamepad1().b().whenBecomesTrue(
                theclawIntake.INSTANCE.open
        );*/
    }
}
