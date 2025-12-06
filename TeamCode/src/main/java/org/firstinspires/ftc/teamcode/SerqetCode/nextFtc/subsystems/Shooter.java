package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class Shooter implements Subsystem {

    /* This creates a public class that allows any opmode access to our SHOOTER system
       All functions of this subsystem are controlled by this one class of code and can be referenced by both auto and tele modes
       through the calling of the predefined Commands below.  Any actions or adjustments to the operation of the INTAKE should take
       place within this single class
    */

    public static final Shooter INSTANCE = new Shooter();       // public INSTANCE of this class to give access

    private Shooter() { }                                       // private object of this class

    // Declare servos and motors
    private final ServoEx servoVertical = new ServoEx("shooter_vertical");
    private final ServoEx servoHorizontal = new ServoEx("shooter_horizontal");

    // TODO - program servos to be mirrored and work in unison
            // TODO - program as servogroup with feedback
    // TODO - verify motor directions
    private MotorGroup shooterGroup = new MotorGroup(
            new MotorEx("left_shooter").reversed(),
            new MotorEx("right_shooter"));

    // PID for the SHOOTER motors

    private ControlSystem controller = ControlSystem.builder()
            .velPid(0.0001, 0, 0)
            .basicFF(0.001,0,0)
            .build();

    // command to call when aiming and shooting action is attempted
    public Command shoot(double launchVelocity, double launchAngle, double aimAngle) {              // feed calculated values into motor control and servos
        return new SetPosition(servoHorizontal,launchAngle).requires(servoHorizontal)                  // servo angle adjustment
                .and(new SetPosition(servoVertical, launchAngle).requires(servoVertical))           // may need a delay here ???
                .and(new RunToVelocity(controller, launchVelocity).requires(this));
    }


    // command to spin up shooter motors
    public Command spinup = new RunToVelocity(controller,.15).requires(this);  // initial spin up command (with delay) that may not be needed

    // command to stop
    public Command stop = new RunToVelocity(controller, 0).requires(this)  // stop flywheels
            .and(new SetPosition(servoHorizontal,0).requires(this)      // reset to default angle
            .and(new SetPosition(servoVertical,0).requires(this))) ;    // reset to default angle


    // this is where actions of this subsystem can be automatically run upon pushing the INIT button
    @Override
    public void initialize(){

    }

    // this is where actions of this subsystem are automatically called during the loop() of the NextFTC opmodes
    // since this class uses a motor control system and commands only define that systems settings, we have to
    // make the system set the motors power during each loop cycle to have an effect on the motor
    @Override
    public void periodic(){
        shooterGroup.setPower(controller.calculate(shooterGroup.getState()));    // update the controller calculations with each loop iteration
    }

}



