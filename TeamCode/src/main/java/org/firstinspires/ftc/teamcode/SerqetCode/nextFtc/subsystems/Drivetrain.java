package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class Drivetrain implements Subsystem {

    public static final Drivetrain INSTANCE = new Drivetrain();

    private Drivetrain(){}
    private final MotorEx frontLeft = new MotorEx("front_left");
    private final MotorEx frontRight = new MotorEx("front_right").reversed();
    private final MotorEx backLeft = new MotorEx("back_left");
    private final MotorEx backRight = new MotorEx("back_right").reversed();

    DriverControlledCommand driverControlled = new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
                // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
    );

    @Override
    public void periodic(){

        // Unsure if this is good practice or not, but it's at least error free
        // as an attempt and simpler than creating a new driveControlled in teleop
        // based on trigger input
        // TODO - test TURBO button based in subsystem
        if(gamepad1.left_trigger > 0.5) {
            driverControlled.setScalar(((gamepad1.left_trigger + 1 ) * 0.5));
        }
        else{
            driverControlled.setScalar(.6);
        }
    }

}
