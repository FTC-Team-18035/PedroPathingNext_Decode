package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.VaultSubsystem;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Variable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import static dev.nextftc.bindings.Bindings.*;

/*  Current MAIN teleop status:

    Drivetrain is active inside MAIN and should be moved to a Subsystem
    Robotcentric is current contol mode

    INTAKE is active
    VAULT is active
    SHOOTER is active

    Control layout (all are gamepad1)  TODO - test and get driver feedback for layout

    left_trigger is variable TurboMode
    right_bumper is INTAKE
    a is SHOOT
*/



@TeleOp(name = "Serqet Lift/Intake/Vault TeleOp")

public class SerqetNextFtcTeleOp extends NextFTCOpMode {
    public SerqetNextFtcTeleOp() {
        addComponents(
             new SubsystemComponent(LiftSubsystem.INSTANCE),    // enable LIFT system
             new SubsystemComponent(IntakeSubsystem.INSTANCE),    // enable INTAKE system
             new SubsystemComponent(VaultSubsystem.INSTANCE),    // enable VAULT system
             new SubsystemComponent(Shooter.INSTANCE),    // enable SHOOTER system

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
    private IMUEx pinpoint = new IMUEx("pinpoint", Direction.UP, Direction.FORWARD).zeroed();    // needed for Pedro field centric
    public double dtScale = 0.5; // Drivetrain scalar variable to set default to half power

    // Actions to take when opmode is INITIALIZED
    @Override
    public void onInit()  {                     // set LIFT to hold the hold the lift
        final Command holdClear = LiftSubsystem.INSTANCE.holdClear;
        holdClear.schedule();
    }

    // Actions to take/ button bindings when START is pressed
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
        driverControlled.setScalar(dtScale);
        driverControlled.schedule();

        // Bind TurboMode to left trigger
        // this is trickier since the trigger(supplier) isn't a boolean
        Variable<Float> left_trigger = variable(() -> gamepad1.left_trigger);
        Button turboButton = left_trigger.asButton(value -> value > 0.5); // true when left trigger is positive.
        turboButton                                                     // TURBO mode but it's variable
                .whenTrue(() -> dtScale = (gamepad1.left_trigger))      // Lambda command to set trigger value to be the drivetrain scalar value
                .whenBecomesFalse(() -> dtScale = 0.5);                 // default to half power


        // Bind shooting actions to gampad1.a
        button(() -> gamepad1.a)
                .whenTrue(Shooter.INSTANCE.spinup)               // may not be helpful - a delay in shoot command (Subsystem level) may be best
                .whenTrue(Shooter.INSTANCE.shoot(5,.1, 0)
                        .and(VaultSubsystem.INSTANCE.outtake))
                // TODO - how to trigger Limelight read/Trajectory calculation and pass
                // TODO - Have PinPoint hold position Pedro?
                .whenBecomesFalse(VaultSubsystem.INSTANCE.stop)
                .whenFalse(Shooter.INSTANCE.stop);

        // Bind LIFT activation to button
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(LiftSubsystem.INSTANCE.toHigh);        // Parking action to raise lift

        // Bind INTAKE actions to button
        button(() -> gamepad1.right_bumper)
                .whenTrue(IntakeSubsystem.INSTANCE.run
                    .and(VaultSubsystem.INSTANCE.intake))               // activate INTAKE and VAULT for getting artifacts
                .whenFalse(IntakeSubsystem.INSTANCE.stop
                    .and(VaultSubsystem.INSTANCE.stop));                // de-activate INTAKE and VAULT
    }



    @Override
    public void onUpdate() {            // code to run during loop()
        // in loop(), or in NextFTC, onUpdate():
        BindingManager.update();        // this is what checks for the gamepad input during loop

    }

    @Override
    public void onStop(){               // code to run once on stop()
        // in stop(), or in NextFTC, onStop():
        BindingManager.reset();         // this is just housekeeping at the end of teleOp
    }

    // TODO - add sensor methods here (pinpoint/Limelight)
    // private void Limelight3A() {
    //
    // }
    // private void pinpoint() {
    //
    // }
}
