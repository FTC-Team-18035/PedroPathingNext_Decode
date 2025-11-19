package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsytems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class Shooter implements Subsystem {

    public static final Shooter INSTANCE = new Shooter();

    private Shooter() { }

    MotorGroup shooterGroup = new MotorGroup(
            new MotorEx("left_shooter"),
            new MotorEx("right_shooter").reversed()
    );

    public Command shoot = new SetPower(shooterGroup, .5).requires(this);

    public Command stop = new SetPower(shooterGroup, 0).requires(this);
}
