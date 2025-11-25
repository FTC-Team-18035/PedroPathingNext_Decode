package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import org.firstinspires.ftc.teamcode.SerqetCode.Trajectory;

public class Shooter implements Subsystem {

    public static final Shooter INSTANCE = new Shooter();
    public static double targetDistance = 0;
    private Shooter() { }

    // Declare servos and motors
    private final ServoEx servoVertical = new ServoEx("shooter_vertical");
    private final ServoEx servoHorizontal = new ServoEx("shooter_horizontal");
    private MotorGroup shooterGroup = new MotorGroup(
            new MotorEx("left_shooter"),
            new MotorEx("right_shooter").reversed());

    // PID for the shooter motors
    // TODO - program servos
    // TODO - verify motor directions
    // TODO - tune PID values
    private ControlSystem controller = ControlSystem.builder()
            .velPid(0.0001, 0, 0.001)
            .basicFF(0.001,0,0)
            .build();

    // command to call when aiming and shooting action is attempted
    public Command shoot(double launchVelocity, double launchAngle, double aimAngle) {                   // feed calculated values into motor control and servos
        return new SetPosition(servoHorizontal,aimAngle).requires(servoHorizontal)          // servo angle adjustment
                .and(new SetPosition(servoVertical, launchAngle).requires(servoVertical))   // may need a delay here ???
                .then(new RunToVelocity(controller, launchVelocity).requires(this));
    }
    // command to spin up shooter motors
    public Command spinup = new RunToVelocity(controller,.5).requires(this).thenWait(0.1);  // initial spin up command (with delay) that may not be neede
    // command to stop
    public Command stop = new RunToVelocity(controller, 0).requires(this);  // stop shooter motors by setting controller goal to 0

    @Override
    public void periodic(){
        shooterGroup.setPower(controller.calculate(shooterGroup.getState()));    // update the controller calculations with each loop iteration
    }

}



