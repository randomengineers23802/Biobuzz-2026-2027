package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSide;

@Autonomous
public class BlueRightSideRightPark extends RightSide {
    private final Pose start = poseFactory.of(100, 134.312, 270);
    private final Pose path1 = poseFactory.of(100, 128, 90);
    private final Pose path2 = poseFactory.of(134.5, 132, 90);
    private final Pose path2Control1 = poseFactory.of(120.8612, 112.3705, 0);
    private final Pose path3Start = poseFactory.of(134.5, 132, 90);
    private final Pose path3 = poseFactory.of(94.5, 21, 270);
    private final Pose path3Control1 = poseFactory.of(97.5144, 96.2865, 0);
    private final Pose path3Control2 = poseFactory.of(128.8546, 56.0483, 0);
    private final Pose path4Start = poseFactory.of(94.5, 21, 270);
    private final Pose path4 = poseFactory.of(124.5, 93, 0);
    private final Pose path4Control1 = poseFactory.of(118.6349, 33.589, 0);
    private final Pose path4Control2 = poseFactory.of(87.3586, 100.7756, 0);
    private final Pose path5Start = poseFactory.of(124.5, 93, 0);
    private final Pose path5 = poseFactory.of(94.5, 21, 270);
    private final Pose path5Control1 = poseFactory.of(107.116, 99.1057, 0);
    private final Pose path5Control2 = poseFactory.of(115.9136, 38.9573, 0);
    private final Pose path6 = poseFactory.of(94.5, 10.5, 270);
    private final Pose point7 = poseFactory.of(94.5, 16, -90);
    private final Pose path8 = poseFactory.of(126, 50, 270);
    private final Pose path8Control1 = poseFactory.of(101.5172, 52.8291, 0);


    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(path1, path2Control1, path2).constant(path2);
    }

    public Path path3() {
        return Paths.curve(path3Start, path3Control1, path3Control2, path3).linear(path3Start, path3);
    }

    public Path path4() {
        return Paths.curve(path4Start, path4Control1, path4Control2, path4).linear(path4Start, path4, 0.5);
    }

    public Path path5() {
        return Paths.curve(path5Start, path5Control1, path5Control2, path5).linear(path5Start, path5);
    }

    public Path path6() {
        return Paths.line(path5, path6).constant(path6);
    }

    public Path path7() {
        return Paths.line(path6, point7).reverseTangent();
    }

    public Path path8() {
        return Paths.curve(point7, path8Control1, path8).reverseTangent();
    }

    public Path pullOutPath4() {
        return Paths.line(path4, poseFactory.of(114.5, 93, 0)).reverseTangent();
    }

    public Path goInPath4() {
        return Paths.line(poseFactory.of(114.5, 93, 0), path4).tangent();
    }

    public Path pullOutPath6() {
        return Paths.line(path6, path5).reverseTangent();
    }

    public Path goInPath6() {
        return Paths.line(path5, path6).tangent();
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.BLUE);
    }
}
