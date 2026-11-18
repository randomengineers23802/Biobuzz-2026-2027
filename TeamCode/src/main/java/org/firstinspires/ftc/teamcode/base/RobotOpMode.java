package org.firstinspires.ftc.teamcode.base;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.control.Robot;
import org.firstinspires.ftc.teamcode.control.Passthrough;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;
    protected Follower follower;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        robot = new Robot(hardwareMap, follower);
    }

    @Override
    public void loop() {
        follower.update();
        robot.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Passthrough.pose = follower.pose();
    }
}
