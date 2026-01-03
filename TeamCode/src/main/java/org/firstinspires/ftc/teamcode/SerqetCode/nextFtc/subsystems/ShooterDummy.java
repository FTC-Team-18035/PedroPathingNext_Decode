package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;


import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class ShooterDummy implements Subsystem {

    public static final ShooterDummy INSTANCE = new ShooterDummy();
    private ShooterDummy() { }
    public static double closePower = 0;
    public static double farPower = 0;
    public static double closeAimAngle = 0;
    public static double farAimAngle = 0;

    private ServoGroup servoGroup = new ServoGroup(
            new ServoEx("horizontal_servo"),
            new ServoEx("vertical_servo"));

    private MotorGroup shooterGroup = new MotorGroup(
            new MotorEx("left_shooter"),
            new MotorEx("right_shooter").reversed());

    public Command shootClose = new SetPower(shooterGroup, closePower).requires(shooterGroup)
            .and(new SetPosition(servoGroup, closeAimAngle).requires(servoGroup));

    public Command shootFar = new SetPower(shooterGroup, farPower).requires(shooterGroup)
            .and(new SetPosition(servoGroup, farAimAngle).requires(servoGroup));

    public Command stopMotors = new SetPower(shooterGroup, 0).requires(servoGroup);
}
