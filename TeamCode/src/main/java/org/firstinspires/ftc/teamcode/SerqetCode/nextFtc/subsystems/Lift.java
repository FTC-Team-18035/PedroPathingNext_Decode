package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class Lift implements Subsystem {

    /* This creates a public class that allows any opmode access to our LIFT system
       All functions of this subsystem are controlled by this one class of code and can be referenced by both auto and tele modes
       through the calling of the predefined Commands below.  Any actions or adjustments to the operation of the INTAKE should take
       place within this single class
    */

    public static final Lift INSTANCE = new Lift();     // public INSTANCE of this class to give access

    private Lift() { }                                  // private object of this class

    private MotorEx motor = new MotorEx("lift");  // get lift motor for private use of this class

    // TODO - tune PID - may be able to make constructor public static??
        private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.015, 0, 0.0002)
            .elevatorFF(0.04)
            .build();


    // this command is designed to run from the beginning of the opmodes and hold the LIFT baseplate up off the field
    public Command holdClear = new RunToPosition(controlSystem, 0).requires(this); // hold plate up for driving

    // TODO - determine correct encoder value for full and safe climbing
    // this command is the singular function of the LIFT system during endgame to climb high and allow parking under Serqet
    public Command toHigh = new RunToPosition(controlSystem, 1200).requires(this); // engage lift for parking

    // this is where actions of this subsystem can be automatically run upon pushing the INIT button
    @Override
    public void initialize() {

        // TODO - test if this actually holds the LIFT system up at the start and during teleop movement
        //new RunToPosition(controlSystem, 0);                   // set the control system to have a goal of 0
        //motor.setPower(controlSystem.calculate(motor.getState()));  // and maintain ground clearance for teleop movement

    }

    // this is where actions of this subsystem are automatically called during the loop() of the NextFTC opmodes
    // since this class uses a motor control system and commands only define that systems settings, we have to
    // make the system set the motors power during each loop cycle to have an effect on the motor
    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }


}
