package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Park;

@Autonomous
public class BluePark extends Park {
    private final Pose start = poseFactory.of(133.9839, 12.8946, 180);
    private final Pose path1 = poseFactory.of(129.7661, 21.0698, 180);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.BLUE);
    }

}
