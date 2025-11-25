package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.variable;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.VaultSubsystem;

import java.util.List;

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


@TeleOp(name = "Serqet Lift/Intake/Vault/ Limelight TeleOp")

public class SerqetNextFtcTeleOpLimelight extends NextFTCOpMode {
    public SerqetNextFtcTeleOpLimelight() {
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
    private Limelight3A limelight;
    private final MotorEx frontLeft = new MotorEx("front_left").reversed();
    private final MotorEx frontRight = new MotorEx("front_right");
    private final MotorEx backLeft = new MotorEx("back_left").reversed();
    private final MotorEx backRight = new MotorEx("back_right");
    private IMUEx pinpoint = new IMUEx("pinpoint", Direction.UP, Direction.FORWARD).zeroed();    // needed for Pedro field centric
    public double dtScale = 0.5; // Drivetrain scalar variable to set default to half power
    public int aprilTag;

    // Actions to take when opmode is INITIALIZED
    @Override
    public void onInit()  {                     // set LIFT to hold the hold the lift
        // final Command holdClear = LiftSubsystem.INSTANCE.holdClear;
        // holdClear.schedule();
        llInitialize();
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
                .whenBecomesTrue(Shooter.INSTANCE.spinup)               // may not be helpful - a delay in shoot command (Subsystem level) may be best
                .whenTrue(Shooter.INSTANCE.shoot(5,.1, 0)
                        .then(VaultSubsystem.INSTANCE.outtake))
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
    private void llInitialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0); // Change depending on Limelight tuning

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

     }

    private int llScan() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();

            telemetry.addData("Botpose", botpose.toString());

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                aprilTag = fr.getFiducialId();
                // telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }

        } else {
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();

        return aprilTag;
    }
    // private void pinpoint() {
    //
    // }
}
