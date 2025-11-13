package org.firstinspires.ftc.teamcode.nextFTC.subsytems;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class theclawIntake implements Subsystem{
    // Hannah and Grace
    public static final theclawIntake INSTANCE = new theclawIntake();
    private theclawIntake() { }

    @Override
    public void initialize() {
        new SetPosition(servo, 0).requires(this);
    }
    private ServoEx servo = new ServoEx("Intake Claw");
    public Command open = new SetPosition(servo, 0).requires(this);
    public Command close = new SetPosition(servo, .45).requires(this);
}