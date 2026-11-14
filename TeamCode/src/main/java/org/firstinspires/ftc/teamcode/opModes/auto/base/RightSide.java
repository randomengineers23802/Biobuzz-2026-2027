package org.firstinspires.ftc.teamcode.opModes.auto.base;


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

    @Override
    protected Command autoRoutine() {
        return sequential(
                parallel(
                        waitMs(1500),   //flywheel spin up time
                        robot.descore.down
                ),
                robot.shoot(),
                follow(path1()),
                //waitUntilTimeout(robot.transfer.isFull(), 1000),
                race(
                        waitMs(2000),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                follow(path2()),
                robot.shoot(),
                follow(path3()),
                //waitUntilTimeout(robot.transfer.isFull(), 1000),
                race(
                        waitMs(2000),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                robot.shoot(),
                followWithActionAt(path4(), 0.3, robot.descore.up),
                //waitUntilTimeout(robot.transfer.isFull(), 1000),
                race(
                        waitMs(2000),
                        waitUntil(() -> robot.transfer.isFull())
                ),
                follow(path5()),
                robot.shoot(),
                follow(path6())
        );
    }
}
