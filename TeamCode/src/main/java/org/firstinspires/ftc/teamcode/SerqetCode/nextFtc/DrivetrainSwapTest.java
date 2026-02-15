package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems.Drivetrain;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Disabled
public class DrivetrainSwapTest extends NextFTCOpMode {

    public DrivetrainSwapTest() {
        addComponents(

                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeft = new MotorEx("front_left");
    private final MotorEx frontRight = new MotorEx("front_right").reversed();
    private final MotorEx backLeft = new MotorEx("back_left");
    private final MotorEx backRight = new MotorEx("back_right").reversed();

    public String mainDrive = "Drivetrain";
    public int driveValue = 0;

    public DriverControlledCommand driverControlled;

    @Override
    public void onInit() {

    }

    @Override
    public void onStartButtonPressed() {
        switch (driveValue) {
            case 0:
                driverControlled = new MecanumDriverControlled(
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight,
                        Gamepads.gamepad1().leftStickY().negate(), //.map((x) -> {return x * dtScalar;}),
                        Gamepads.gamepad1().leftStickX(), //.map((x) -> {return x * dtScalar;}),
                        Gamepads.gamepad1().rightStickX() //.map((x) -> {return x * dtScalar;})
                        // new HolonomicMode.FieldCentric(imu)  // needed for Pedro field centric
                );
                button(() -> gamepad1.dpad_right)
                        .whenBecomesTrue(() -> driveValue = 1);
                break;

            case 1:
                driverControlled = new PedroDriverControlled(
                        Gamepads.gamepad1().leftStickY().negate(),
                        Gamepads.gamepad1().leftStickX().negate(),
                        Gamepads.gamepad1().rightStickX().negate(),
                        false
                );
                button(() -> gamepad1.dpad_right)
                        .whenBecomesTrue(() -> driveValue = 0);
                break;
        }

        driverControlled.schedule();

    }

    @Override
    public void onUpdate() {
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
