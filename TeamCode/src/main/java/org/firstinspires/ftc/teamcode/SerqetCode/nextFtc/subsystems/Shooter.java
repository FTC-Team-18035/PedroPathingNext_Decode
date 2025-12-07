package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
@Configurable
public class Shooter implements Subsystem {

    /* This creates a public class that allows any opmode access to our SHOOTER system
       All functions of this subsystem are controlled by this one class of code and can be referenced by both auto and tele modes
       through the calling of the predefined Commands below.  Any actions or adjustments to the operation of the INTAKE should take
       place within this single class
    */

    public static final Shooter INSTANCE = new Shooter();       // public INSTANCE of this class to give access

    public static PIDFController ShooterController;
    public double servoAngle;
    private Shooter() { }                                       // private object of this class

    // Declare servos and motors
    public static double sp = 0.008, si = 0, sd = 0, sf = 0.00055;
    public static double shooterTarget = 100;
    private final double shooter_ticks_in_degrees = 28;   // set this for specific motors completed 12/6

    private final ServoEx servoVertical = new ServoEx("shooter_vertical");
    private final ServoEx servoHorizontal = new ServoEx("shooter_horizontal");
    private final MotorEx leftShooter = new MotorEx("left_shooter");
    private final MotorEx rightShooter = new MotorEx("right_shooter").reversed();


    // TODO - FUTURE REFINEMENT: program as servogroup with feedback
    // private final ServoGroup aimGroup = new ServoGroup(
    //      new FeedbackServoEx("shooter_vertical" , 0 , 0.0 )      // unsure of this-need to research
    //      new FeedbackServoEx("shooter_horizontal" , 0 , 0.0 ));  // FeedbackServoEx documentation




    // PID for the SHOOTER motors

   /* private ControlSystem controller = ControlSystem.builder()
            .velPid(0.008, 0, 0)
            .basicFF(0.00055,0,0)
            .build(); */

    // command to call when aiming and shooting action is attempted
   /* public Command shoot(double launchVelocity, double launchAngle, double aimAngle) {              // feed calculated values into motor control and servos
        return new SetPosition(servoHorizontal, checkIfSafe(launchAngle)).requires(servoHorizontal)                  // servo angle adjustment // TODO FUTURE make the shooter servos into a group to make sure that they run at the same time
                .and(new SetPosition(servoVertical, checkIfSafe(launchAngle)).requires(servoVertical))           // may need a delay here ???
                .and(new Command() {
                    @Override
                    public boolean isDone() {
                        shooterTarget = launchVelocity;
                        return false;
                    }
                }/*RunToVelocity(controller, launchVelocity)).requires(this));
    }*/
    public Command shoot(double launchVelocity, double launchAngle, double aimAngle) {
        return new SetPosition(servoHorizontal, checkIfSafe(launchAngle)).requires(servoHorizontal)
                .and(new SetPosition(servoVertical, checkIfSafe(launchAngle)).requires(servoVertical))
                .and(new LambdaCommand().setStart(() -> shooterTarget = launchVelocity));
    }

    // command to spin up shooter motors - probably will be DEPRECATED when motor tuning is completed
    //public Command spinup = new RunToVelocity(controller,1).requires(this);  // initial spin up command (with delay) that may not be needed

    public Command stop = new LambdaCommand()
            .setStart(() -> {
                shooterTarget = 0;
                    });

    // command to stop
  /*  public Command stop = new RunToVelocity(controller, 0).requires(this)  // stop flywheels
            .and(new SetPosition(servoHorizontal,.1809).requires(this)      // reset to min angle
            .and(new SetPosition(servoVertical,.1809).requires(this))) ;    // reset to min angle
    */

    // this is where actions of this subsystem can be automatically run upon pushing the INIT button
    @Override
    public void initialize(){
        ShooterController = new PIDFController(sp, si, sd, sf);
    }

    // this is where actions of this subsystem are automatically called during the loop() of the NextFTC opmodes
    // since this class uses a motor control system and commands only define that systems settings, we have to
    // make the system set the motors power during each loop cycle to have an effect on the motor
    @Override
    public void periodic(){
        //shooterGroup.setPower(ShooterController.calculate(shooterGroup.getState()));    // update the controller calculations with each loop iteration

        updateShoot(shooterTarget);

    }

    public double checkIfSafe(double servoAngle) {
        double newAngle = servoAngle;
        if (servoAngle >= .2306) {
            newAngle = .2306;
        }
        if (servoAngle <= .1809) {
            newAngle = .1809;
        }
        return newAngle;
    }

    public void updateShoot(double target) {
        ShooterController.setPIDF(sp, si, sd, sf);          // use new PIDF values

        double shooterVel = leftShooter.getVelocity();      // read velocity

        double sPID = ShooterController.calculate(shooterVel, target);   // calculate path to new velocity target

        double shooterFF = Math.cos(Math.toRadians(target / shooter_ticks_in_degrees)) * sf;  // more math

        double shooterPower = sPID + shooterFF;             // calculate corrected power value

        double error = target - shooterVel;

        leftShooter.setPower(shooterPower);                 // set power to motors
        rightShooter.setPower(-shooterPower);               // TODO - verify motor directions !!!

        ActiveOpMode.telemetry().addData("Left Shooter Power", leftShooter.getPower());
        ActiveOpMode.telemetry().addData("Target Velocity", target);
        ActiveOpMode.telemetry().update();
    }

}



