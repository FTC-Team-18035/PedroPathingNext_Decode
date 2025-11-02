package org.firstinspires.ftc.teamcode.nextFTC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.theclaw;
import org.firstinspires.ftc.teamcode.theclawIntake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;

import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous(name = "NextFTC Autonomous Program Java")
public class Attempt1NextFtcAutonomous extends NextFTCOpMode {
        public Attempt1NextFtcAutonomous() {
            addComponents(
                    new SubsystemComponent(theclaw.INSTANCE, theclawIntake.INSTANCE, CRservoSubsytemTest.INSTANCE),
                    BulkReadComponent.INSTANCE
            );
        }

        private Command autonomousRoutine() {
            return new SequentialGroup(
                    CRservoSubsytemTest.INSTANCE.go,
                    new Delay(2),
                    new ParallelGroup(
                            theclaw.INSTANCE.close,
                            theclawIntake.INSTANCE.close
                    ),
                    new Delay(.5),
                    new ParallelGroup(
                            theclaw.INSTANCE.open,
                            CRservoSubsytemTest.INSTANCE.stop,
                            theclawIntake.INSTANCE.open

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
