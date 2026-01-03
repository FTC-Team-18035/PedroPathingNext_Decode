package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SerqetCode.Trajectory;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Intake;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Lift;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Vault;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

/*
  *  Current MAIN teleop status:
  *
  *  Drivetrain is active inside MAIN and should be moved to a Subsystem
  *  Robotcentric is current control mode
  *
  *  INTAKE is active
  *  VAULT is active
  *  SHOOTER is active
  *  LIFT is deactivated        // TODO - create software or mechanical solution for lift stabilization until endgame
  *  LIMELIGHT is operating and in feature development  TODO - verify function and calibrate with Apriltag reading measurements
  *  PINPOINT is functioning IMU  TODO - verify pinpoint setup and accuracy
  *
  *  Control layout  TODO - test and get driver feedback for layout
  *
  *  GAMEPAD1
  *  left_trigger is TurboMode
  *  left_bumper is INTAKE EJECT
  *  right_bumper is INTAKE
  *  dpad_up if activate LIFT
  *  a is SHOOT
  *
  *  GAMEPAD2               TODO - make these features automatic in code when proven working
  *  a is start limelight   // these should happen during autonomous for targeted INTAKE and SHOOTING and NAVIGATION by Apriltags
  *  b is stop limelight    // and automatically after successful INTAKE/and scoring during teleop for power savings
  *  x is reset pinpoint position // this should happen after successful limelight Apriltag localization to update robots global position
  */


@TeleOp(name = "1st LL/PP features")

public class TestMAIN extends NextFTCOpMode {
    public TestMAIN() {
        addComponents(
                // new SubsystemComponent(Lift.INSTANCE),    // enable LIFT system
                //  DEACTIVATED FOR 12-6 TESTING
                new SubsystemComponent(Intake.INSTANCE),    // enable INTAKE system
                new SubsystemComponent(Vault.INSTANCE),    // enable VAULT system
                new SubsystemComponent(Shooter.INSTANCE),    // enable SHOOTER system

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final String telemetryValue = null;

    // Names for DECODE season robot SERQET

    private Limelight3A limelight;    // configured later as part of llIntialize method
    private GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    private final MotorEx frontLeft = new MotorEx("front_left");
    private final MotorEx frontRight = new MotorEx("front_right").reversed();
    private final MotorEx backLeft = new MotorEx("back_left");
    private final MotorEx backRight = new MotorEx("back_right").reversed();
    public double dtScalar = 0.5; // Drivetrain scalar variable to set default to half power - could be calibrated to ANY VALUE upon testing
                                  // TODO - decide if Turbo feature is a hindrance and DEPRECATE if so
    public int aprilTag;
    double defaultDistance = 200;
    double llDistance = defaultDistance;  // default distance of shooter in cm ???

    // Actions to take when opmode is INITIALIZED
    @Override
    public void onInit() {
        // initialize limelight
        llInitialize();

    }

    // Actions to take/ button bindings when START is pressed
    @Override
    public void onStartButtonPressed() {
        // Optional: 100ms delay to allow Pinpoint v2 to initialize
        try {Thread.sleep(100);} catch (InterruptedException e) {Thread.currentThread().interrupt();        }
        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - TODO this should be the place you are starting the robot from and should pass from auto via BLACKBOARD
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

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

        // ****** GAMEPAD1 controls ******

        // Bind TURBO button to gamepad1.left_trigger  TODO - this button replaces the previous IF statement in onUpdate loop - 90% confidence on this implementation
        button(() -> gamepad1.left_trigger > 0.1 )      // TODO - if unsuccessful comment out and uncomment IF statement in onUpdate loop
                .whenBecomesTrue(() -> {dtScalar = 1;})                        // scale to full power
                .whenBecomesFalse(() -> {dtScalar = 0.5;});                    // default to half power

        // Bind shooting actions to gamepad1.a
        button(() -> gamepad1.a)
                // .whenBecomesTrue(Shooter.INSTANCE.spinup)  DEPRECATED UPON DISCUSSION WITH MECHANICAL
                .whenBecomesTrue(() -> {
                    double[] trajPair = Trajectory.Calculate(llDistance);               // this is limelight informed trajectory method calculation call
                    new Delay(0.1);                                                // delay added to the sequence just for fun
                    Shooter.INSTANCE.shoot(trajPair[0], trajPair[1], 0)        // this is calculated shooting minus any fine tuning for horizontal aim
                            .and(Vault.INSTANCE.outtake); })                            // TODO - test feature and develop fine tuning

                // TODO - Have PinPoint help hold position for actively stabilized shooting
                // possibly get position-switch to Pedro controlled to maintain-when shooting is finished then
                // switch to driverControlled
                .whenBecomesFalse(Vault.INSTANCE.stop.and(Shooter.INSTANCE.stop))
                .whenFalse(Shooter.INSTANCE.stop);

        /*  DEACTIVATED FOR 12-6 testing
        // Bind LIFT activation to button
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(Lift.INSTANCE.toHigh);        // Parking action to raise lift
        */

        // Bind INTAKE actions to button
        button(() -> gamepad1.right_bumper)
                .whenTrue(Intake.INSTANCE.run.and(Vault.INSTANCE.intake))           // activate INTAKE and VAULT for getting artifacts
                .whenFalse(Intake.INSTANCE.stop.and(Vault.INSTANCE.stop));          // de-activate INTAKE and VAULT

        // TODO - determine if we need to move the vault while ejecting
        button(() -> gamepad1.left_bumper)
                .whenTrue(Intake.INSTANCE.eject                                     // reverse INTAKE to eject excess artififacts
                        .and(Vault.INSTANCE.eject))
                .whenFalse(Intake.INSTANCE.stop
                        .and(Vault.INSTANCE.stop));

        // ****** GAMEPAD2 controls ******  TODO - none of these gamepad2 features have been tested - all should produce telemetry information

        // gamepad 2 starts/stops limelight in this test opMode
        button(() -> gamepad2.a)
                .whenBecomesTrue(() -> limelight.start())
                .whenTrue(() -> llScan());                      // activating limelight should produce informed and calculated settings to the shooter

        button(() -> gamepad2.b)
                .whenTrue(() -> limelight.pause());

        // x button to reset robot position to 0,0,0
        button(() -> gamepad2.x)                                // a pinpoint reset at the center of the field should allow for FTC coordinate plotting
                .whenBecomesTrue(() -> pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));

    }  // end of onStartButtonPressed

        @Override
        public void onUpdate() {            // code to run during loop()

            BindingManager.update();        // this is what checks for the gamepad input during loop

            pinpoint.update();              // calls for the IMU update of data each loop
            Pose2D pose2D = pinpoint.getPosition();

            // Telemetry info here          currently all telemetry is created in the limelight methods below
            double[] trajPairTelemetry = Trajectory.Calculate(llDistance);

            telemetry.addData("trajectory feedback", trajPairTelemetry[0]);
            telemetry.addData("angle feedback", trajPairTelemetry[1]);
            telemetry.addData("botPose", pose2D.toString());
            telemetry.update();

        /* DEPRECATED FEATURE but saved in case turbo button via button binding fails  TODO - test and remove if binding is successful
        dtScalar = (gamepad1.left_trigger > 0.1) ? 1:0.5;      // TURBO button via brute force
        */
    }

    @Override
    public void onStop() {               // code to run once on stop()
        BindingManager.reset();         // this is just housekeeping at the end of teleOp
    }

    // limelight initialization sequence
    private void llInitialize() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(8); // Change depending on Limelight tuning

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

    }

    // limelight method to get the current april tag being read - this is mostly just copy/paste of sample method
    private int llScan() {                      // TODO - calibrate limelight
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();                                 // advanced Apriltag pose estimation
            limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));   // aided by pinpoint heading reading
            llDistance = getDistance(llResult.getTa());                                 // this is the method call to calculate distance from limelight area reading
            telemetry.addData("Calculated Distance" , llDistance);               // as given in the video
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("LLBotpose", botPose.toString());                 // this pose estimation is limelight guided for X,Y and pinpoint for heading

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                aprilTag = fr.getFiducialId();
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }
        else {                          // TODO - determine the value and validity of this feature
            llDistance = defaultDistance;             // this could be a place holder value for a default shooting value if limelight readings are unavailable (ie another robot is in the way)
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();

        return aprilTag;
    }

    // code copy from Youtube video for distance calculation from valid limelight result's current Apriltag target
    public double getDistance(double ta ){
        double scale = 30660.00 ;   // TODO - measure to get scale via YOUTUBE https://youtu.be/Ap1lBywv00M?si=ol1Yp7WaJYLBue6D
        return ( scale/ ta );       // this number is a rounded off usage of his similar configuration readings

    }

    /*
      Here are the configuration value settings for the pinpoint IMU
    */
    private void configurePinpoint () {         // copied from Limelight_Pinpoint_Fixed from 12-6 testing

        pinpoint.setOffsets(-94.9706, -70.70786, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,   // verified 12-6
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();
    }
}  // end of NextFTCOpMode
