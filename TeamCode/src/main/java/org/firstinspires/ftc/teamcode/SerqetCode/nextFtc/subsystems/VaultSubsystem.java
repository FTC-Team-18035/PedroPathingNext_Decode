package org.firstinspires.ftc.teamcode.SerqetCode.nextFtc.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class VaultSubsystem implements Subsystem {
    public static final VaultSubsystem INSTANCE = new VaultSubsystem();
    private VaultSubsystem() {}
    private CRServoEx vaultFeed = new CRServoEx( "vault_feed");
    private ServoEx vaultRelease = new ServoEx( "vault_release");

    public Command intake = new SetPosition(vaultRelease, 1).requires(this).and(
            new SetPower(vaultFeed, -1).requires(this));
    public Command outtake = new SetPosition(vaultRelease,0).requires(this).and(
           new SetPower(vaultFeed,-1).requires(this));

    public Command run = new SetPower(vaultFeed, -1).requires(this);
    public Command stop = new SetPower(vaultFeed, 0).requires(this);

    public Command open = new SetPosition(vaultRelease, 0).requires(this);

    public Command close = new SetPosition(vaultRelease, 1).requires(this);



}
