package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Vault implements Subsystem {

    /* This creates a public class that allows any opmode access to our VAULT system
       All functions of this subsystem are controlled by this one class of code and can be referenced by both auto and tele modes
       through the calling of the predefined Commands below.  Any actions or adjustments to the operation of the INTAKE should take
       place within this single class
    */

    public static final Vault INSTANCE = new Vault();     // public INSTANCE of this class to give access
    private Vault() {}                                    // private object of this class

    // servos used by this subsystem
    private CRServoEx vaultFeed = new CRServoEx( "vault_feed");
    private ServoEx vaultRelease = new ServoEx( "vault_release");

    // public commands for actions of this subsystem

    public Command intake = new SetPosition(vaultRelease, 1).requires(this).and(
            new SetPower(vaultFeed, -1).requires(this));

    public Command outtake = new SetPosition(vaultRelease,0).requires(this).and(
           new SetPower(vaultFeed,-1).requires(this));

    // These were first stage commands prior to multi step ones created above
    public Command run = new SetPower(vaultFeed, -1).requires(this);

    public Command stop = new SetPower(vaultFeed, 0).requires(this);

    public Command open = new SetPosition(vaultRelease, 0).requires(this);

    public Command close = new SetPosition(vaultRelease, 1).requires(this);


    // this is where actions of this subsystem can be automatically run upon pushing the INIT button
    @Override
    public void initialize() {

    }

    // this is where actions of this subsystem are automatically called during the loop() of the NextFTC opmodes
    @Override
    public void periodic() {

    }



}
