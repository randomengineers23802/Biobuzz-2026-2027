package org.firstinspires.ftc.teamcode.opModes.auto.base;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.base.AutoOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class RightSideWithPush extends AutoOpMode {
    protected abstract Path path1();
    protected abstract Path path2();
    protected abstract Path path3();
    protected abstract Path path4();
    protected abstract Path path5();
    protected abstract Path path6();
    protected abstract Path path7();
    protected abstract Path path8();
    protected abstract Path pullOutPath2();
    protected abstract Path goInPath2();
    protected abstract Path pullOutPath4();
    protected abstract Path goInPath4();

    @Override
    protected Command autoRoutine() {
        return sequential(
                follow(path1()),
                waitMs(1500),
                robot.shoot(),
                followWithActionAt(path2(), robot.descore.down, 0.3),
                race(
                        waitMs( 1500),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                conditional(
                        () -> robot.transfer.isFull(),
                        instant(() -> {}),
                        sequential(
                                follow(pullOutPath2()),
                                follow(goInPath2()),
                                race(
                                        waitMs(1500),
                                        waitUntil(() -> robot.transfer.isFull())
                                )
                        )
                ),
                follow(path3()),
                waitMs(500),
                robot.shoot(),
                follow(path4()),
                race(
                        waitMs(1500),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                conditional(
                        () -> robot.transfer.isFull(),
                        instant(() -> {}),
                        sequential(
                                follow(pullOutPath4()),
                                follow(goInPath4()),
                                race(
                                        waitMs(1500),
                                        waitUntil(() -> robot.transfer.isFull())
                                )
                        )),
                robot.shoot(),
                race(
                        followWithActionAt(path5(), robot.descore.up, 0.3),
                        waitMs(4000),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                follow(path6()),
                waitMs(500),
                robot.shoot(),
                follow(path7()),
                follow(path8().with(Constants.foresightConfig.maxPathSpeed.at(0.5)))
        );
    }
}
