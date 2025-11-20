package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class IntakeSubsystem implements Subsystem {
        public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
        private IntakeSubsystem() { }

        private MotorEx motor = new MotorEx("intake");

        public Command run = new SetPower(motor, 1).requires(this);
        public Command stop = new SetPower(motor, 0).requires(this);
}
