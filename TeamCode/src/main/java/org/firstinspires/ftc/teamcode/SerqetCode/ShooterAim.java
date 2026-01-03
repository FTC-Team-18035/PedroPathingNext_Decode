package org.firstinspires.ftc.teamcode.SerqetCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class ShooterAim extends OpMode {

    private Servo horizontal;
    private Servo vertical;

    double hPos, vPos;

    @Override
    public void init() {
        // Initialize servos with hardwareMap
        horizontal = hardwareMap.get(Servo.class, "shooter_horizontal");
        vertical = hardwareMap.get(Servo.class, "shooter_vertical");

        // Initialize at max servo angle
        horizontal.setPosition(0.2306);
        vertical.setPosition(0.2306);

        telemetry.addData("Status", "Servos initialized at 0");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Map joystick [-1,1] to servo [0,1]
        //double hPos = (gamepad1.left_stick_y + 1) / 2.0;
        // double vPos = (gamepad1.right_stick_y + 1) / 2.0;


        //if stick > max value times 2 then the position equals max value
        if (gamepad1.left_stick_y > 0.2306 * 2) {
            hPos = 0.2306;
            vPos = 0.2306;
        }
        else{
            hPos = gamepad1.left_stick_y/2;
            vPos = gamepad1.left_stick_y/2;
        }

        horizontal.setPosition(hPos);
        vertical.setPosition(vPos);

        // Telemetry feedback
        telemetry.addData("Horizontal servo", hPos);
        telemetry.addData("Vertical servo", vPos);
        telemetry.addData("Joystick L Y", gamepad1.left_stick_y);
        telemetry.addData("Joystick R Y", gamepad1.right_stick_y);
        telemetry.update();
    }
}
