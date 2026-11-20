package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSideWithPush;

@Autonomous
@Disabled
public class RedRightSideWithPush extends RightSideWithPush {
    private final Pose start = poseFactory.of(59, 9.688, 90);
    private final Pose path1p1 = poseFactory.of(59, 16, 270);
    private final Pose path1p2 = poseFactory.of(59, 12, 270);
    private final Pose path2Start = poseFactory.of(59, 12, 270);
    private final Pose path2 = poseFactory.of(19.5, 49.5, 180);
    private final Pose path2Control1 = poseFactory.of(60.5698, 43.6357, 180);
    private final Pose path3Start = poseFactory.of(19.5, 49.5, 180);
    private final Pose path3 = poseFactory.of(49.5, 123, 90);
    private final Pose path3Control1 = poseFactory.of(46.4856, 47.7135, 180);
    private final Pose path3Control2 = poseFactory.of(15.1454, 87.9517, 180);
    private final Pose path4 = poseFactory.of(49.5, 128, 90);
    private final Pose path5Start = poseFactory.of(49.5, 128, 90);
    private final Pose path5 = poseFactory.of(9.5, 12, 270);
    private final Pose path5Control1 = poseFactory.of(22, 114, 180);
    private final Pose path5Control2 = poseFactory.of(27.5, 37, 180);
    private final Pose path6p1 = poseFactory.of(14, 24, 270);
    private final Pose path6p2Start = poseFactory.of(14, 24, 270);
    private final Pose path6p2 = poseFactory.of(61, 127, 90);
    private final Pose path6p2Control1 = poseFactory.of(30.4133, 39.9033, 180);
    private final Pose path6p2Control2 = poseFactory.of(17.8798, 122.7055, 180);
    private final Pose path7Start = poseFactory.of(61, 127, 90);
    private final Pose path7 = poseFactory.of(12, 62.5843, 270);
    private final Pose path7Control1 = poseFactory.of(29.2592, 121.2389, 180);
    private final Pose path7Control2 = poseFactory.of(28.8019, 61.6302, 180);
    private final Pose path8 = poseFactory.of(12, 97.191, 270);

    public Path path1p1() {
        return Paths.line(start, path1p1).linear(start, path1p1);
    }

    public Path path1p2() {
        return Paths.line(path1p1, path1p2).constant(path1p2);
    }

    public Path path1() {
        return Paths.path(path1p1(), path1p2());
    }

    public Path path2() {
        return Paths.curve(path2Start, path2Control1, path2).linear(path2Start, path2, 0.5);
    }

    public Path path3() {
        return Paths.curve(path3Start, path3Control1, path3Control2, path3).linear(path3Start, path3, 0.5);
    }

    public Path path4() {
        return Paths.line(path3, path4).constant(path4);
    }

    public Path path5() {
        return Paths.curve(path5Start, path5Control1, path5Control2, path5).linear(path5Start, path5);
    }

    public Path path6p1() {
        return Paths.line(path5, path6p1).constant(path6p1);
    }

    public Path path6p2() {
        return Paths.curve(path6p2Start, path6p2Control1, path6p2Control2, path6p2).linear(path6p2Start, path6p2);
    }

    public Path path6() {
        return Paths.path(path6p1(), path6p2());
    }

    public Path path7() {
        return Paths.curve(path7Start, path7Control1, path7Control2, path7).linear(path7Start, path7, 0.5);
    }

    public Path path8() {
        return Paths.line(path7, path8).reverseTangent();
    }

    public Path pullOutPath2() {
        return Paths.line(path2, poseFactory.of(114.5, 94.5, 0)).reverseTangent();
    }

    public Path goInPath2() {
        return Paths.line(poseFactory.of(114.5, 94.5, 0), path2).tangent();
    }

    public Path pullOutPath4() {
        return Paths.line(path4, poseFactory.of(94.5, 26, 270)).reverseTangent();
    }

    public Path goInPath4() {
        return Paths.line(poseFactory.of(94.5, 26, 270), path4).tangent();
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.RED);
    }
}
