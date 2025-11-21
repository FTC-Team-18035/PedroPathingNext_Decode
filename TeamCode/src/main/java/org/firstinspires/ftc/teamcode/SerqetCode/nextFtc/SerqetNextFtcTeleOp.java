package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.VaultSubsystem;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Variable;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;


@TeleOp(name = "Serqet Lift/Intake/Vault TeleOp")
public class SerqetNextFtcTeleOp extends NextFTCOpMode {
    public SerqetNextFtcTeleOp() {
        addComponents(
             new SubsystemComponent(LiftSubsystem.INSTANCE),    // enable LIFT system
             new SubsystemComponent(IntakeSubsystem.INSTANCE),    // enable INTAKE system
             new SubsystemComponent(VaultSubsystem.INSTANCE),    // enable VAULT system
                // new SubsystemComponent(ShooterSubSystem.INSTANCE),    // enable SHOOTER system

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private String telemetryValue = null;

    // Names for DECODE season robot SERQET
    private final MotorEx frontLeft = new MotorEx("front_left").reversed();
    private final MotorEx frontRight = new MotorEx("front_right");
    private final MotorEx backLeft = new MotorEx("back_left").reversed();
    private final MotorEx backRight = new MotorEx("back_right");

    // private IMUEx imu = new IMUEx("imu", Direction.UP, Direction.FORWARD).zeroed()    // needed for Pedro field centric
    public double dtScale = 1;


    @Override
    public void onStartButtonPressed() {
        telemetry.addData("", telemetryValue);
        // enable CHASSIS
        // TODO - determine motor directions for proper operation
        DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
                // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
        );

        Variable<Float> left_trigger = variable(() -> gamepad1.left_trigger);
        Button turboButton = left_trigger.asButton(value -> value > 0.5); // true when left trigger is positive.

        turboButton // TURBO mode but it's variable // change to whiletrue if value does not update
                .whenTrue(() -> dtScale = (gamepad1.left_trigger))
                .whenBecomesFalse(() -> dtScale = 0.5);             // default to half power

        driverControlled.setScalar(dtScale);
        driverControlled.schedule();

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(
                        LiftSubsystem.INSTANCE.toHigh
                );                                          // Bind LIFT activation to button

        //TODO Figure out Trajectory. We need Limelight feedback for trajectory and shooting.
        //TODO Have PinPoint hold position Pedro?
        //TODO Hit Shoot use limelight for feedback, if we get pushed move back, then shoot
        Gamepads.gamepad1().a().whenBecomesTrue(
                        Shooter.INSTANCE.shoot(1)              // Here could be the commands to hold position via Pinpoint and Pedro?
        ).whenBecomesFalse(
                Shooter.INSTANCE.stop
        );                                                  // Bind SHOOTER activation to button

        Gamepads.gamepad1().rightBumper().whenTrue(
                       IntakeSubsystem.INSTANCE.run.and(
                       VaultSubsystem.INSTANCE.intake)
                );                                          // Bind INTAKE/VAULT run operation to button
        Gamepads.gamepad1().rightBumper().whenFalse(
                        IntakeSubsystem.INSTANCE.stop.and(
                        VaultSubsystem.INSTANCE.stop)
                );                                          // Bind INTAKE/VAULT stop operation to button


    }



    @Override
    public void onUpdate() {            // code to run during loop()
        // in loop(), or in NextFTC, onUpdate():
        BindingManager.update();

    }

    @Override
    public void onStop(){               // code to run once on stop()
        // in stop(), or in NextFTC, onStop():
        BindingManager.reset();
    }

}
