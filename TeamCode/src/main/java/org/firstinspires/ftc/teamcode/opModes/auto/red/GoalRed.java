package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Goal;

@Autonomous
@Configurable
public class GoalRed extends Goal {
    @Override
    protected void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(167.500, 176.000),
                                new Pose(134.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(216.5), Math.toRadians(224))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 128.000),
                                new Pose(137.000, 107.000),
                                new Pose(183.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(360), 0.15)
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(183.000, 106.000),
                                new Pose(153.000, 109.000),
                                new Pose(134.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(224))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 128.000),
                                new Pose(132.000, 117.000),
                                new Pose(185.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(400), 0.15)
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(185.000, 106.000),
                                new Pose(152.000, 110.000),
                                new Pose(134.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(400), Math.toRadians(224))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 128.000),
                                new Pose(132.000, 117.000),
                                new Pose(185.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(400), 0.15)
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(185.000, 106.000),
                                new Pose(152.000, 110.000),
                                new Pose(134.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(400), Math.toRadians(224))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(134.000, 128.000),
                                new Pose(132.000, 117.000),
                                new Pose(185.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(400), 0.15)
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(185.000, 106.000),
                                new Pose(152.000, 110.000),
                                new Pose(134.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(400), Math.toRadians(224))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(134.000, 128.000),
                                new Pose(167.000, 118.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(224), Math.toRadians(270))
                .build();
    }

    @Override
    public void init() {
        super.init();
        follower.setStartingPose(new Pose(167.500, 176.000, Math.toRadians(216.5)));
        robot.setAlliance(Alliance.RED);
    }
}
