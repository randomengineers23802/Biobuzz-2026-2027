package org.firstinspires.ftc.teamcode.opModes.auto.base;


import static com.pedropathing.ivy.groups.Groups.sequential;
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
                shoot(),
                followWithActionAt(path1(), 0.2, robot.descore.down),
                waitUntilTimeout(robot.transfer.full(), 1000),
                follow(path2()),
                shoot(),
                follow(path3()),
                waitUntilTimeout(robot.transfer.full(), 1000),
                shoot(),
                followWithActionAt(path4(), 0.3, robot.descore.up),
                waitUntilTimeout(robot.transfer.full(), 1000),
                follow(path5()),
                shoot(),
                follow(path6())
        );
    }
}
