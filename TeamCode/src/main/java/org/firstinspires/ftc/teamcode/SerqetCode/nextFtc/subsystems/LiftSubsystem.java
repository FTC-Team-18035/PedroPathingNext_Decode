package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class LiftSubsystem implements Subsystem {
    public static final LiftSubsystem INSTANCE = new LiftSubsystem();
    private LiftSubsystem() { }
    private MotorEx motor = new MotorEx("lift");  // changed to SERQET name

    // TODO - tune PID
        private ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0, 0, 0)
            .elevatorFF(0)
            .build();
    // TODO - determine correct encoder value
    public Command holdClear = new RunToPosition(controlSystem, 0).requires(this); // hold plate up for driving
    public Command toHigh = new RunToPosition(controlSystem, 1200).requires(this); // engage lift for parking

    @Override
    public void initialize() { motor.setPower(controlSystem.calculate(motor.getState())); }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }
}
