package org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.CRservoSubsytemTest;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclaw;
import org.firstinspires.ftc.teamcode.JD_Code.limelight.nextFTC.subsytems.theclawIntake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Disabled
@Autonomous(name = "NextFTC Autonomous Program Java")
public class Attempt1NextFtcAutonomous extends NextFTCOpMode {
        public Attempt1NextFtcAutonomous() {
            addComponents(
                    new SubsystemComponent(theclaw.INSTANCE, theclawIntake.INSTANCE, CRservoSubsytemTest.INSTANCE),
                    BulkReadComponent.INSTANCE
            );
        }

    @Override
    public void onInit() {
            theclaw.INSTANCE.initialize();
            theclawIntake.INSTANCE.initialize();
    }

    private Command autonomousRoutine() {
            return new SequentialGroup(
                    CRservoSubsytemTest.INSTANCE.go,
                    new Delay(2),
                    CRservoSubsytemTest.INSTANCE.stop,
                    theclaw.INSTANCE.open,
                    new Delay(2),
                    theclaw.INSTANCE.close,
                    theclawIntake.INSTANCE.open,
                    new Delay(2),
                    theclawIntake.INSTANCE.close,
                    new Delay(5),
                    new ParallelGroup(
                            theclaw.INSTANCE.open,
                            theclawIntake.INSTANCE.open,
                            CRservoSubsytemTest.INSTANCE.go
                    ),
                    new Delay(.5),
                    new ParallelGroup(
                            theclaw.INSTANCE.close,
                            CRservoSubsytemTest.INSTANCE.stop,
                            theclawIntake.INSTANCE.close

                    ),
                    new Delay(.5),
                    CRservoSubsytemTest.INSTANCE.go
            );
        }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }
}
