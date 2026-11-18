package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSide;

@Autonomous
public class BlueRightSide extends RightSide {
    private final Pose start = poseFactory.of(85, 134.312, 270);
    private final Pose path1p1 = poseFactory.of(85, 128, 90);
    private final Pose path1p2 = poseFactory.of(85, 132, 90);
    private final Pose path2Start = poseFactory.of(85, 132, 90);
    private final Pose path2 = poseFactory.of(124.5, 94.5, 0);
    private final Pose path2Control1 = poseFactory.of(83.4302, 100.3643, 0);
    private final Pose path3Start = poseFactory.of(124.5, 94.5, 0);
    private final Pose path3 = poseFactory.of(94.5, 21, 270);
    private final Pose path3Control1 = poseFactory.of(97.5144, 96.2865, 0);
    private final Pose path3Control2 = poseFactory.of(128.8546, 56.0483, 0);
    private final Pose path4 = poseFactory.of(94.5, 16, 270);
    private final Pose path5Start = poseFactory.of(94.5, 16, 270);
    private final Pose path5 = poseFactory.of(134.5, 132, 90);
    private final Pose path5Control1 = poseFactory.of(122, 30, 0);
    private final Pose path5Control2 = poseFactory.of(116.5, 107, 0);
    private final Pose path6p1 = poseFactory.of(130, 120, 90);
    private final Pose path6p2Start = poseFactory.of(130, 120, 90);
    private final Pose path6p2 = poseFactory.of(83, 17, 270);
    private final Pose path6p2Control1 = poseFactory.of(113.5867, 104.0967, 0);
    private final Pose path6p2Control2 = poseFactory.of(126.1202, 21.2945, 0);
    private final Pose path7 = poseFactory.of(127, 21, 270);

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
        return Paths.line(path6p2, path7).constant(path7);
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
        robot.setAlliance(Alliance.BLUE);
    }
}
