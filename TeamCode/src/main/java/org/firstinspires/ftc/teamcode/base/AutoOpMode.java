package org.firstinspires.ftc.teamcode.base;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;

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
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        super.loop();

    }

    protected CommandBuilder follow(Path path) {
        return PedroCommands.follow(follower, path);
    }

    protected Command fastFollow(Path path) {
        return follow(path).setDone(() -> follower.completion() > 0.95 && follower.completion() != 1.0);
    }

//    protected Command timeFollow(PathChain path, double seconds) { //needs work
//        return deadline(
//                waitMs(seconds * 1000),
//                follow(path)
//        );
//    }
//
//    protected Command timeFollow(PathChain path, double seconds) { //needs work
//        Timer timer = new Timer();
//        return follow(path).setStart(timer::resetTimer).setDone(() -> timer.getElapsedTimeSeconds() >= seconds);
//    }
}
