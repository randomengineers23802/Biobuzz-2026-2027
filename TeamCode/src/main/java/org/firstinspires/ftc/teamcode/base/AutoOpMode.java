package org.firstinspires.ftc.teamcode.base;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoOpMode extends RobotOpMode {
    protected abstract Command autoRoutine();
    protected final PoseFactory poseFactory = PoseFactory.degrees();

    @Override
    public void init() {
        super.init();
        Scheduler.reset(); //maybe move to robot op mode
        robot.descore.up.schedule();
        robot.stopper.close.schedule();
    }

    @Override
    public void init_loop() {
        follower.update();
        robot.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
        robot.intake.on.schedule();
        robot.transfer.on.schedule();
    }

    protected CommandBuilder follow(Path path) {
        return PedroCommands.follow(follower, path);
    }

    protected Command followWithActionAt(Path path, double completion, Command action) {
        return parallel(
                follow(path),
                sequential(
                        waitUntil(() -> follower.completion() >= completion),
                        action
                )
        );
    }

    protected Command timeFollow(Path path, double ms) {
        return race(
                waitMs(ms),
                PedroCommands.follow(follower, path)
        );
    }

    protected Command waitUntilTimeout(boolean waitUntil, double ms) {
        return race(
                waitMs(ms),
                waitUntil(() -> waitUntil)
        );
    }

}
