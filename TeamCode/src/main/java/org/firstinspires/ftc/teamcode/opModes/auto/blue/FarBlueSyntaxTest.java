package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import static com.pedropathing.api.Paths.line;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Far;

@Autonomous
public class FarBlueSyntaxTest extends Far {

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56, 36, 180);
    private final Pose point2 = poseFactory.of(95, 87, -127.4054);
    private final Pose point3 = poseFactory.of(87, 70, 64.7989);

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return line(path1, point2).reverseTangent();
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    @Override
    public void init() {
        super.init();
        follower.setPose(start);
        robot.setAlliance(Alliance.BLUE);
        PIDF test = new PIDF(5,5,5,5,5,5,50, PIDF.Mode.GENERIC);
    }
}
