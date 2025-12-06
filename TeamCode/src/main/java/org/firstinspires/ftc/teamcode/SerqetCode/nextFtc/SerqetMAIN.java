package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SerqetCode.Trajectory;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Intake;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Lift;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Vault;

import dev.nextftc.bindings.BindingManager;
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

@TeleOp(name = "Serqet TURBO Test")

public class SerqetMAIN extends NextFTCOpMode {

/*  Current MAIN teleop status:

    Drivetrain is active inside MAIN and should be moved to a Subsystem
    Robotcentric is current control mode

    INTAKE is active
    VAULT is active
    SHOOTER is active
    LIFT is active

    Control layout (all are gamepad1)  TODO - test and get driver feedback for layout

    left_trigger is TurboMode
    right_bumper is INTAKE
    left_bumper is REVERSE INTAKE
    a is SHOOT
    dpad up is LIFT
*/


    public SerqetMAIN() {
            addComponents(
                    new SubsystemComponent(Lift.INSTANCE),    // enable LIFT system
                    new SubsystemComponent(Intake.INSTANCE),    // enable INTAKE system
                    new SubsystemComponent(Vault.INSTANCE),    // enable VAULT system
                    new SubsystemComponent(Shooter.INSTANCE),    // enable SHOOTER system

                    BulkReadComponent.INSTANCE,
                    BindingsComponent.INSTANCE
            );
        }

        private String telemetryValue = null;

        // Names for DECODE season robot SERQET
        // DRIVETRAIN motor directions verified 11/25/25
        private final MotorEx frontLeft = new MotorEx("front_left");
        private final MotorEx frontRight = new MotorEx("front_right").reversed();
        private final MotorEx backLeft = new MotorEx("back_left");
        private final MotorEx backRight = new MotorEx("back_right").reversed();
        private IMUEx pinpoint = new IMUEx("pinpoint", Direction.UP, Direction.FORWARD).zeroed();    // needed for Pedro field centric
        private double dtScalar = 0.5;     // factor to reduce drivetrain input controls unless TURBO button is pressed
        public double lldistance = 5;   // distance WILL BE set by limelight calculations

        // Actions to take when opmode is INITIALIZED
        @Override
        public void onInit() {
            // set LIFT to hold the hold the lift
            // TODO - this function has not been verified although motor direction has
            //final Command holdClear = Lift.INSTANCE.holdClear;
            //holdClear.schedule();
        }

        // Actions to take/ button bindings when START is pressed
        @Override
        public void onStartButtonPressed() {
            telemetry.addData("", telemetryValue);

            // enable DRIVETRAIN
            DriverControlledCommand driverControlled = new MecanumDriverControlled(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    Gamepads.gamepad1().leftStickY().negate().map((x) -> {return x * dtScalar;}),
                    Gamepads.gamepad1().leftStickX().map((x) -> {return x * dtScalar;}),
                    Gamepads.gamepad1().rightStickX().map((x) -> {return x * dtScalar;})
                    // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
            );
            driverControlled.schedule();

            // TODO - test TURBO button feature moved to onUpdate() and mapping function in the driverControlled constructor

            // Bind shooting actions to gamepad1.a
            button(() -> gamepad1.a)
                    .whenBecomesTrue(Shooter.INSTANCE.spinup)             // may not be helpful - a delay in shoot command (Subsystem level) may be best
                    .whenTrue(
                        () -> {double[] trajPair = Trajectory.Calculate(1000);  // Testing in TestMAIN on 12/6
                        Shooter.INSTANCE.shoot(trajPair[0], trajPair[1], 0)
                        .and(Vault.INSTANCE.outtake);})


                    .whenBecomesFalse(Vault.INSTANCE.stop)
                    .whenFalse(Shooter.INSTANCE.stop);

            // Bind LIFT activation to button
            button(() -> gamepad1.dpad_up)
                    .whenBecomesTrue(Lift.INSTANCE.toHigh);        // Parking action to raise lift

            // Bind INTAKE actions to button
            button(() -> gamepad1.right_bumper)
                    .whenTrue(Intake.INSTANCE.run
                            .and(Vault.INSTANCE.intake))               // activate INTAKE and VAULT for getting artifacts
                    .whenFalse(Intake.INSTANCE.stop
                            .and(Vault.INSTANCE.stop));                // de-activate INTAKE and VAULT

            // TODO - determine if we need to move the vault while ejecting
            button(() -> gamepad1.left_bumper)
                    .whenTrue(Intake.INSTANCE.eject
                            .and(Vault.INSTANCE.eject))
                    .whenFalse(Intake.INSTANCE.stop
                            .and(Vault.INSTANCE.stop));
        }


        @Override
        public void onUpdate() {            // code to run during loop()

            BindingManager.update();                // this is what checks for the gamepad input during loop
            if (gamepad2.a) {

            }
            if (gamepad1.left_trigger > 0.1) {      // TURBO button via brute force
                dtScalar = 1;
            }
            else {
                dtScalar = 0.5;
            }

        }

        @Override
        public void onStop() {               // code to run once on stop()

            BindingManager.reset();         // this is just housekeeping at the end of teleOp
        }

        // Testing in TestMAIN on 12/6
        // private void Limelight3A() {
        //
        // }
        // private void pinpoint() {
        //
        // }
}
