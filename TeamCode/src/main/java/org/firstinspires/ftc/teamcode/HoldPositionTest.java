package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Hold Position Test")
public class HoldPositionTest extends LinearOpMode { // The name of the class (This has to match the file name)
    private double frontLeftPower = 0;     // declare motor power variable
    private double backLeftPower = 0;      // declare motor power variable
    private double frontRightPower = 0;    // declare motor power variable
    private double backRightPower = 0;     // declare motor power variable
    private double denominator = 1;        // declare motor power calculation variable
    private int    precision = 2;          // chassis motor power reduction factor 1

    private double HoldXPosition = 0;   // Saves the current X position
    private double HoldYPosition = 0;   // Saves the current Y position

    private boolean HoldPositionSet = false;

    @Override
    public void runOpMode() {
        //*********************************** MOTORS ************************************************
        DcMotor FrontRight = hardwareMap.dcMotor.get("Front Right");   // Chub Port 0 // Gamepad 1
        DcMotor BackRight = hardwareMap.dcMotor.get("Back Right");     // Chub Port 1 // Left Stick For Moving
        DcMotor FrontLeft = hardwareMap.dcMotor.get("Front Left");     // Chub Port 2 // Right Stick For Turning
        DcMotor BackLeft = hardwareMap.dcMotor.get("Back Left");       // Chub Port 3

        // Dead wheel feedback port 1 (Back Right) and 2 (Front Right)

        //****************************** REVERSE MOTORS *****************************************************

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //****************************** SET MOTORS TO BRAKE MODE *****************************************************
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // Sets the motor to be locked when stopped
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);     // Sets the motor to be locked when stopped
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);    // Sets the motor to be locked when stopped

        waitForStart();

        while (opModeIsActive()) {



            //************************** DRIVE CONTROLS **************************************************
            // check for driving input
            double y = -gamepad1.left_stick_y;         // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1;    // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;        // Measures turning
            if (gamepad1.right_trigger >= 0.75) {      // Checks if the Right Trigger was pressed and if so it continues the stuff in the brackets
                y = gamepad1.left_stick_y;             // Remember, this is reversed!
                x = -gamepad1.left_stick_x * 1.1;      // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;           // Measures turning
            }

            if (gamepad1.left_trigger > .25) {
                double CurrentYPosition = FrontRight.getCurrentPosition();       // Y position
                double CurrentXPosition = BackRight.getCurrentPosition();         // X position

                if (!HoldPositionSet) {
                    HoldXPosition = CurrentXPosition;
                    HoldYPosition = CurrentYPosition;
                    HoldPositionSet = true;
                }

                if (CurrentXPosition < HoldXPosition) {
                    x = -.50 * 1.1;
                }
                else if (CurrentXPosition > HoldXPosition) {
                    x = .50 * 1.1;
                }
                else { x = 0; }

                if (CurrentYPosition < HoldYPosition) {
                    y = -.50;
                }
                else if (CurrentYPosition > HoldYPosition) {
                    y = .50;
                }
                else { y = 0; }

            }
            else { HoldPositionSet = false; }

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);    // calculate motor movement math and adjust according to lift height or manual precision mode selection

            // check for Turbo or Precision Mode
            if (gamepad1.left_bumper) {         // Left bumper is being pressed
                precision = 1;                  // set speed to full power - TURBO MODE
            } else if (gamepad1.right_bumper) { // Right bumper is being pressed
                precision = 4;                  // set speed to 1/4 power - PRECISION MODE
            } else {
                precision = 2;                  // reset default speed to half power
            }

            // calculate motor power
            denominator = denominator * precision;          // this adjusts motor speed to the desired precision level
            frontLeftPower = (y + x + rx) / denominator;    // Does math to convert stick movement to motor powers
            backLeftPower = (y - x + rx) / denominator;     // Does math to convert stick movement to motor powers
            frontRightPower = (y - x - rx) / denominator;   // Does math to convert stick movement to motor powers
            backRightPower = (y + x - rx) / denominator;    // Does math to convert stick movement to motor powers

            // issue Drive Wheels motor power
            FrontLeft.setPower(frontLeftPower);    // Sets the front left wheel's power
            BackLeft.setPower(backLeftPower);     // Sets the back left wheel's power
            FrontRight.setPower(frontRightPower);  // Sets the front right wheel's power
            BackRight.setPower(backRightPower);   // Sets the back right wheel's power

            telemetry.update();

        }
    }
}
