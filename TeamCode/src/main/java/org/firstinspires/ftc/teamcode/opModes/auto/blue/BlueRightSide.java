package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSide;

@Autonomous
public class BlueRightSide extends RightSide {
    private final Pose start = poseFactory.of(85, 134.312, 0);
    private final Pose path1 = poseFactory.of(124.5, 94.5, 0);
    private final Pose point2Start = poseFactory.of(124.5, 94.5, 0);
    private final Pose point2 = poseFactory.of(94.5, 20, 270);
    private final Pose point2Control1 = poseFactory.of(120, 47, 0);
    private final Pose point3 = poseFactory.of(94.5, 16, 270);
    private final Pose point4Start = poseFactory.of(94.5, 16, 270);
    private final Pose point4 = poseFactory.of(134.5, 132, 90);
    private final Pose point4Control1 = poseFactory.of(122, 30, 0);
    private final Pose point4Control2 = poseFactory.of(116.5, 107, 0);
    private final Pose point5 = poseFactory.of(109, 130, 90);
    private final Pose point6 = poseFactory.of(130, 50, 90);

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
        robot.setAlliance(Alliance.BLUE);
    }

}
