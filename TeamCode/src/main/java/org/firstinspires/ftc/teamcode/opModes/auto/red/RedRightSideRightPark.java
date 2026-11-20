package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.RightSide;

@Autonomous
public class RedRightSideRightPark extends RightSide {
    private final Pose start = poseFactory.of(44, 9.688, 90);
    private final Pose path1 = poseFactory.of(44, 16, 270);
    private final Pose path2 = poseFactory.of(9.5, 12, 270);
    private final Pose path2Control1 = poseFactory.of(23.1388, 31.6295, 180);
    private final Pose path3Start = poseFactory.of(9.5, 12, 270);
    private final Pose path3 = poseFactory.of(49.5, 123, 90);
    private final Pose path3Control1 = poseFactory.of(46.4856, 47.7135, 180);
    private final Pose path3Control2 = poseFactory.of(15.1454, 87.9517, 180);
    private final Pose path4Start = poseFactory.of(49.5, 123, 90);
    private final Pose path4 = poseFactory.of(19.5, 51, 180);
    private final Pose path4Control1 = poseFactory.of(25.3651, 110.411, 180);
    private final Pose path4Control2 = poseFactory.of(56.6414, 43.2244, 180);
    private final Pose path5Start = poseFactory.of(19.5, 51, 180);
    private final Pose path5 = poseFactory.of(49.5, 123, 90);
    private final Pose path5Control1 = poseFactory.of(36.884, 44.8943, 180);
    private final Pose path5Control2 = poseFactory.of(28.0864, 105.0427, 180);
    private final Pose path6 = poseFactory.of(49.5, 133.5, 90);
    private final Pose point7 = poseFactory.of(49.5, 128, 90);
    private final Pose path8 = poseFactory.of(18, 94, 90);
    private final Pose path8Control1 = poseFactory.of(42.4828, 91.1709, 180);

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
        return Paths.line(path4, poseFactory.of(29.5, 51, 180)).reverseTangent();
    }

    public Path goInPath4() {
        return Paths.line(poseFactory.of(29.5, 51, 180), path4).tangent();
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
        robot.setAlliance(Alliance.RED);
    }
}
