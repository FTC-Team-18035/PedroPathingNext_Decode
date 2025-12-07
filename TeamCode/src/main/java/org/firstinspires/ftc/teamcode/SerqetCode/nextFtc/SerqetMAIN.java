package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// import org.firstinspires.ftc.teamcode.SerqetCode.Trajectory;
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
                        // Shooter button binding (gamepad1.a) - persistent command while held
                        button(() -> gamepad1.a)
                                .whenTrue(() -> {
                                        double[] trajPair = calculateTrajectory(210);
                                        Shooter.INSTANCE.shootPersistent(trajPair[0], trajPair[1], 0).and(Vault.INSTANCE.outtake).schedule();
                                })
                                .whenFalse(() -> Shooter.INSTANCE.stop.schedule());

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
                        BindingManager.update();
                        // TURBO button: scales drivetrain
                        dtScalar = (gamepad1.left_trigger > 0.1) ? 1 : 0.5;

                        // Telemetry for shooter
                        double[] trajPairTelemetry = calculateTrajectory(500);
                        double shooterTargetVelocity = trajPairTelemetry[0];
                        telemetry.addData("Trajectory X (target ticks/s)", shooterTargetVelocity);
                        telemetry.addData("Trajectory Y (angle)", trajPairTelemetry[1]);
                        // If possible, get actual shooter velocity from subsystem (example shown, adjust as needed):
                        // double actualVelocity = Shooter.INSTANCE.getActualVelocity();
                        // telemetry.addData("Shooter Actual Velocity", actualVelocity);
                        telemetry.update();
                }

        @Override
        public void onStop() {               // code to run once on stop()

            BindingManager.reset();         // this is just housekeeping at the end of teleOp
        }


                // Local replacement for Trajectory.Calculate
                private double[] calculateTrajectory(double targetDistance) {
                        double distance = targetDistance;
                        double g = 980.0; // cm/s^2 magnitude of acceleration due to gravity
                        double y0 = 53.0; // cm, target final height in goal
                        double m = Math.min(-1.5, (-200.0 - y0) / distance); // The SLOPE with which an artifact will enter the goal with. NOT AN ANGLE
                        double a = -(y0 / Math.pow(distance, 2)) + (m / distance); // The "a" value in the parabola equation
                        double b = ((2.0 * y0) / distance) - m; // The "b" value in the parabola equation
                        double rawlaunchAngle = Math.atan(b); // The launch angle in radians
                        double launchVelocity = ((1.0 / Math.cos(rawlaunchAngle)) * Math.sqrt(g / (2.0 * a)))* 1.23787177960363;
                        double launchAngle = Math.toDegrees(rawlaunchAngle) * .00392157;
                        return new double[] { launchVelocity, launchAngle };
                }
}
