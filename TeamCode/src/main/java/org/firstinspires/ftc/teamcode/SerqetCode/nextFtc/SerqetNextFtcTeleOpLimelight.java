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
import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.CRservoSubsytemTest;
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
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

/*  Current MAIN teleop status:

    Drivetrain is active inside MAIN and should be moved to a Subsystem
    Robotcentric is current control mode

    INTAKE is active
    VAULT is active
    SHOOTER is active
    LIFT is active

    Control layout  TODO - test and get driver feedback for layout

    GAMEPAD1
    left_trigger is variable TurboMode
    right_bumper is INTAKE
    a is SHOOT

    GAMEPAD2
    a is start limelight
    b is stop limelight
    x is reset position
*/


@TeleOp(name = "Serqet Lift/Intake/Vault TeleOp Limelight")

public class SerqetNextFtcTeleOpLimelight extends NextFTCOpMode {
    public SerqetNextFtcTeleOpLimelight() {
        addComponents(
                new SubsystemComponent(Lift.INSTANCE),    // enable LIFT system
                new SubsystemComponent(Intake.INSTANCE),    // enable INTAKE system
                new SubsystemComponent(Vault.INSTANCE),    // enable VAULT system
                new SubsystemComponent(Shooter.INSTANCE),    // enable SHOOTER system
                // new SubsystemComponent(CRservoSubsytemTest.INSTANCE),  // testing purposes only

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private String telemetryValue = null;

    // Names for DECODE season robot SERQET

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    // CHASSIS motor directions verified 11/25/25
    private final MotorEx frontLeft = new MotorEx("front_left");
    private final MotorEx frontRight = new MotorEx("front_right").reversed();
    private final MotorEx backLeft = new MotorEx("back_left");
    private final MotorEx backRight = new MotorEx("back_right").reversed();
    public double dtScalar = 0.6; // Drivetrain scalar variable to set default to half+ power
    public int aprilTag;

    public double llDistance = 0;

    // Actions to take when opmode is INITIALIZED
    @Override
    public void onInit() {                     // set LIFT to hold the hold the lift
        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - TODO this should be the place you are starting the robot from and should pass from auto via BLACKBOARD
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        // initalize limelight
        llInitialize();

        /*  Deprecated and function moved to subsystem level
        final Command holdClear = Lift.INSTANCE.holdClear;
        holdClear.schedule();
        */

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
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
                // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
        );
        driverControlled.setScalar(dtScalar);
        driverControlled.schedule();

        // TODO - test TURBO button feature moved to onUpdate()

        // Bind shooting actions to gamepad1.a
        button(() -> gamepad1.a)
                .whenBecomesTrue(Shooter.INSTANCE.spinup)
                .whenTrue(() -> {
                    double[] trajPair = Trajectory.Calculate(llDistance /* in mm */);
                    new Delay(0.1);                                                 //
                    Shooter.INSTANCE.shoot(trajPair[0], trajPair[1], 0)
                            .and(Vault.INSTANCE.outtake); })

                // TODO - how to trigger Limelight read/Trajectory calculation and pass
                // TODO - Have PinPoint hold position
                // possibly get position-switch to Pedro controlled to maintain-when shooting is finished then
                // switch to driverControlled
                .whenBecomesFalse(Vault.INSTANCE.stop.and(Shooter.INSTANCE.stop))
                .whenFalse(Shooter.INSTANCE.stop);

        // Bind LIFT activation to button
        button(() -> gamepad1.dpad_up)
                .whenBecomesTrue(Lift.INSTANCE.toHigh);        // Parking action to raise lift

        // Bind INTAKE actions to button
        button(() -> gamepad1.right_bumper)
                .whenTrue(Intake.INSTANCE.run.and(CRservoSubsytemTest.INSTANCE.go))
                //.and(Vault.INSTANCE.intake))           // activate INTAKE and VAULT for getting artifacts
                .whenFalse(Intake.INSTANCE.stop.and(CRservoSubsytemTest.INSTANCE.stop));
        //.and(Vault.INSTANCE.stop));                // de-activate INTAKE and VAULT

    }

    @Override
    public void onUpdate() {            // code to run during loop()

        BindingManager.update();        // this is what checks for the gamepad input during loop

        if (gamepad2.x) {
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();                      // calls for the IMU update of data each loop
        Pose2D pose2D = pinpoint.getPosition();

        if (gamepad2.a) {                        // gamepad 2 starts/stops limelight in this test opMode
            limelight.start();
            llScan();
        }

        if (gamepad2.b) {
            limelight.pause();
        }

    }

    @Override
    public void onStop() {               // code to run once on stop()
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

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
            limelight.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));
            llDistance = getDistance(llResult.getTa());
            telemetry.addData("Calculated Distance", llDistance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());

            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = llResult.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                aprilTag = fr.getFiducialId();
                // telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
        }
        else {
            llDistance=5;
            telemetry.addData("Limelight", "No data available");
        }

        telemetry.update();

        return aprilTag;
    }
    public double getDistance(double ta ){
        double scale = 30000.00 ;   // TODO - measure to get scale
        double llDistance = ( scale/ ta );
        return llDistance;
    }

    /*
      Here is the configuration value settings for the pinpoint IMU
    */
    private void configurePinpoint () {
            /*
             *  Set the odometry pod positions relative to the point that you want the position to be measured from.
             *
             *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
             *  Left of the center is a positive number, right of center is a negative number.
             *
             *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
             *  Forward of center is a positive number, backwards is a negative number.
             */
            pinpoint.setOffsets(-0, -41.2, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
            // Eli provided 11-30
            /*
             * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
             * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
             * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
             * number of ticks per unit of your odometry pod.  For example:
             *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
             */
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            /*
             * Set the direction that each of the two odometry pods count. The X (forward) pod should
             * increase when you move the robot forward. And the Y (strafe) pod should increase when
             * you move the robot to the left.
             */
            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,   // TODO - check directions
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);

            /*
             * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
             * The IMU will automatically calibrate when first powered on, but recalibrating before running
             * the robot is a good idea to ensure that the calibration is "good".
             * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
             * This is recommended before you run your autonomous, as a bad initial calibration can cause
             * an incorrect starting value for x, y, and heading.
             */
            pinpoint.resetPosAndIMU();
    }
}