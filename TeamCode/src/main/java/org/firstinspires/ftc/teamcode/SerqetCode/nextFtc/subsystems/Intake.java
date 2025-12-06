package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

        /* This creates a public class that allows any opmode access to our INTAKE system
           All functions of this subsystem are controlled by this one class of code and can be referenced by both auto and tele modes
           through the calling of the predefined Commands below.  Any actions or adjustments to the operation of the INTAKE should take
           place within this single class
        */

        public static final Intake INSTANCE = new Intake();     // public INSTANCE of this class to give access
        private Intake() { }                                    // private object of this class

        private MotorEx motor = new MotorEx("intake");    // this assigns our robot config motor named "intake" to this subsystem's control

        // TODO - assign eject to a button in the teleop modes - TestMAIN 12/6 implemented for testing

        // this command is called by NextFTCOpModes to run the INTAKE @ negative power
        public Command eject = new SetPower(motor, -1).requires(this);

        // this command is called by NextFTCOpModes to run the INTAKE @ full power
        public Command run = new SetPower(motor, 1).requires(this);

        // this command is called by NextFTCOpModes to stop the INTAKE
        public Command stop = new SetPower(motor, 0).requires(this);


        // this is where actions of this subsystem can be automatically run upon pushing the INIT button
        @Override
        public void initialize() {

        }

        // this is where actions of this subsystem are automatically called during the loop() of the NextFTC opmodes
        @Override
        public void periodic() {

        }

}
