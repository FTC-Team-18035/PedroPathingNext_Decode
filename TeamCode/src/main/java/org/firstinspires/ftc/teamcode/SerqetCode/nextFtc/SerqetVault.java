/*package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@TeleOp(name = "SerqetVault")
public class SerqetVault implements Subsystem {
    public static final SerqetVault INSTANCE = new SerqetVault();

    private SerqetVault() { }
    CRServoEx crServoEx = new CRServoEx("CRServoJoe");                                     //declares a CRServo named CRServoJoe
    ServoEx servoEx = new ServoEx("ServoJim");                                            //declares a Servo named ServoJim
    public Command open = new SetPosition(servoEx,1).requires(this);       //sets the position of ServoJim to 1

    public Command close = new SetPosition(servoEx,0).requires(this);      //sets the position of ServoJim to 0

    public Command on = new SetPower(crServoEx,1).requires(this);         //sets the position of CRServoJoe to 1
    public Command off = new SetPower(crServoEx,0).requires(this);        //sets the position of CRServoJoe to 0
}*/
