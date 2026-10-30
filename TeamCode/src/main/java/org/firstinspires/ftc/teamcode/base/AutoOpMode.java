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
        Scheduler.reset();
        robot.descore.up();
        robot.stopper.close();
    }

    @Override
    public void init_loop() {
        follower.update();
        robot.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
        robot.intake.on();
        robot.transfer.on();
    }



    protected CommandBuilder follow(Path path) {
        return PedroCommands.follow(follower, path);
    }

    protected Command shoot() {
        return sequential(
                robot.stopper.open,
                waitMs(700),
                robot.stopper.close
                );
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

//    protected Command timeFollow(Path path, double seconds) {
//        return deadline(
//                waitMs(seconds * 1000.0),
//                follow(path)
//        );
//    }

    protected Command timeFollow(Path path, double seconds) {
        return deadline(
                waitMs(seconds * 1000.0),
                PedroCommands.follow(follower, path)
        );
    }

    protected Command waitUntilTimeout(boolean waitUntil, double ms) {
        return race(
                waitMs(ms),
                waitUntil(() -> waitUntil)
        );
    }

//    protected Command timeFollow(Path path, double seconds) { //needs work
//        Timer timer = new Timer();
//        return follow(path).setStart(timer::reset).setDone(() -> timer.seconds() >= seconds);
//    }
}
