package org.firstinspires.ftc.teamcode.opModes.auto.blue;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Far;

@Autonomous
@Configurable
public class FarBlue extends Far {
    @Override
    protected void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(72.000, 8.563),

                                new Pose(73.125, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(295))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(73.125, 50.000),
                                new Pose(69.125, 64.000),
                                new Pose(6, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(295), Math.toRadians(180), 0.2)

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(6, 61.000),

                                new Pose(73.125, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(298))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(73.125, 50.000),
                                new Pose(69.125, 88.000),
                                new Pose(6, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(298), Math.toRadians(180), 0.2)

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(6, 85.000),

                                new Pose(73.125, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(298))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(73.125, 50.000),

                                new Pose(3, 11.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(298), Math.toRadians(180))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 11.5),

                                new Pose(73.125, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(298))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(73.125, 50.000),

                                new Pose(3, 11.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(298), Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 11.5),

                                new Pose(73.125, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(298))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(73.125, 50.0),

                                new Pose(50.0, 18.0) //other side
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(298), Math.toRadians(180))

                .build();
    }

    @Override
    public void init() {
        super.init();
        follower.setStartingPose(new Pose(72.000, 8.5625, Math.toRadians(0)));
        robot.setAlliance(Alliance.BLUE);
    }
}