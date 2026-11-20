package org.firstinspires.ftc.teamcode.opModes.auto.base;


import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.*;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.base.AutoOpMode;

public abstract class RightSide extends AutoOpMode {
    protected abstract Path path1();
    protected abstract Path path2();
    protected abstract Path path3();
    protected abstract Path path4();
    protected abstract Path path5();
    protected abstract Path path6();
    protected abstract Path path7();
    protected abstract Path path8();
    protected abstract Path pullOutPath4();
    protected abstract Path goInPath4();
    protected abstract Path pullOutPath6();
    protected abstract Path goInPath6();

    @Override
    protected Command autoRoutine() {
        return sequential(
                follow(path1()),
                waitMs(1500),
                robot.shoot(),
                race(
                        follow(path2()),
                        waitMs( 3000),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                follow(path3()),
                waitMs(500),
                robot.shoot(),
                race(
                        followWithActionAt(path4(), robot.descore.down, 0.2),
                        waitMs(3000)
                ),
                race(
                        waitUntil(() -> robot.transfer.isFull()),
                        waitMs(1500)
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
                        )
                ),
                follow(path5()),
                robot.shoot(),
                race(
                        follow(path6()),
                        waitMs(2000)
                ),
                race(
                        waitUntil(() -> robot.transfer.isFull()),
                        waitMs(1500)
                ),
                conditional(
                        () -> robot.transfer.isFull(),
                        instant(() -> {}),
                        sequential(
                                follow(pullOutPath6()),
                                race(
                                        follow(goInPath6()),
                                        waitMs(1500),
                                        waitUntil(() -> robot.transfer.isFull())
                                )
                        )
                ),
                follow(path7()),
                waitMs(500),
                robot.shoot(),
                followWithActionAt(path8(), robot.descore.up, 0.2)
        );
    }
}
