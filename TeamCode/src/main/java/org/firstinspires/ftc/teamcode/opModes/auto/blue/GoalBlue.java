package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Goal;

@Autonomous
@Configurable
public class GoalBlue extends Goal {
    @Override
    protected void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(24.500, 176.000),
                                new Pose(58.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(323.5), Math.toRadians(316))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.000, 128.000),
                                new Pose(55.000, 107.000),
                                new Pose(9.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(180), 0.15)
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.000, 106.000),
                                new Pose(39.000, 109.000),
                                new Pose(58.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(316))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.000, 128.000),
                                new Pose(60.000, 117.000),
                                new Pose(7.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(140), 0.15)
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(7.000, 106.000),
                                new Pose(40.000, 110.000),
                                new Pose(58.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(316))
                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.000, 128.000),
                                new Pose(60.000, 117.000),
                                new Pose(7.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(140), 0.15)
                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(7.000, 106.000),
                                new Pose(40.000, 110.000),
                                new Pose(58.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(316))
                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(58.000, 128.000),
                                new Pose(60.000, 117.000),
                                new Pose(7.000, 106.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(140), 0.15)
                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(7.000, 106.000),
                                new Pose(40.000, 110.000),
                                new Pose(58.000, 128.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(316))
                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(58.000, 128.000),
                                new Pose(25.000, 118.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(270))
                .build();
    }

    @Override
    public void init() {
        super.init();
        follower.setStartingPose(new Pose(24.500, 176.000, Math.toRadians(323.5)));
        robot.setAlliance(Alliance.BLUE);
    }
}