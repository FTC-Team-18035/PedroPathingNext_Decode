package org.firstinspires.ftc.teamcode.nextFTC.subsytems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.powerable.SetPower;

public class CRservoSubsytemTest implements Subsystem {
    public static final CRservoSubsytemTest INSTANCE = new CRservoSubsytemTest();
    private CRservoSubsytemTest() { }

    private CRServoEx servo = new CRServoEx("testcr");

    public Command go = new SetPower(servo, 1).requires(this);
    public Command stop = new SetPower(servo, 0).requires(this);
}
