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

    @Override
    protected Command autoRoutine() {
        return sequential(
                follow(path1()),
                follow(path2()),
                follow(path3()),
                follow(path4()),
                follow(path5())
        );
    }
}
