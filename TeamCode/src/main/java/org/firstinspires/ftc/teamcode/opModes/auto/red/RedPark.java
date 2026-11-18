package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Park;

@Autonomous
public class RedPark extends Park {
    private final Pose start = poseFactory.of(10.0161, 131.1054, 0);
    private final Pose path1 = poseFactory.of(14.2339, 122.9302, 0);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.RED);
    }

}
