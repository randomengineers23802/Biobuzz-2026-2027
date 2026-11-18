package org.firstinspires.ftc.teamcode.opModes.auto.base;


import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.base.AutoOpMode;

public abstract class Park extends AutoOpMode {
    protected abstract Path path1();

    @Override
    protected Command autoRoutine() {
        return sequential(
                follow(path1())
        );
    }
}
