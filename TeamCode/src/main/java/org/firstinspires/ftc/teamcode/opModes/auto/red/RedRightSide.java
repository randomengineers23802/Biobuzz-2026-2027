package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSide;

@Autonomous
public class RedRightSide extends RightSide {
    private final Pose start = poseFactory.of(59, 9.688, 180);
    private final Pose path1 = poseFactory.of(19.5, 49.5, 180);
    private final Pose point2Start = poseFactory.of(19.5, 49.5, 180);
    private final Pose point2 = poseFactory.of(49.5, 124, 90);
    private final Pose point2Control1 = poseFactory.of(24, 97, 180);
    private final Pose point3 = poseFactory.of(49.5, 128, 90);
    private final Pose point4Start = poseFactory.of(49.5, 128, 90);
    private final Pose point4 = poseFactory.of(9.5, 12, 270);
    private final Pose point4Control1 = poseFactory.of(22, 114, 180);
    private final Pose point4Control2 = poseFactory.of(27.5, 37, 180);
    private final Pose point5 = poseFactory.of(35, 14, 270);
    private final Pose point6 = poseFactory.of(14, 94, 270);

    public Path path1() {
        return Paths.line(start, path1).constant(path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).constant(point3);
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return Paths.line(point4, point5).constant(point5);
    }

    public Path path6() {
        return Paths.line(point5, point6).constant(point6);
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.RED);
    }

}
