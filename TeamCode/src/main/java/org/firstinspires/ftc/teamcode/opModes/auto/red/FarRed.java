package org.firstinspires.ftc.teamcode.opModes.auto.red;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.opModes.auto.base.Far;

@Autonomous
@Configurable
public class FarRed extends Far {
    @Override
    protected void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(120.000, 8.563),

                                new Pose(118.875, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(245))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.875, 50.000),
                                new Pose(122.875, 64.000),
                                new Pose(186, 61.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(360), 0.2)

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(186, 61.000),

                                new Pose(118.875, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(242))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(118.875, 50.000),
                                new Pose(122.875, 88.000),
                                new Pose(186, 85.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(242), Math.toRadians(360), 0.2)

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(186, 85.000),

                                new Pose(118.875, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(242))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.875, 50.000),

                                new Pose(189, 11.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(242), Math.toRadians(360))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(189.000, 11.5),

                                new Pose(118.875, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(242))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.875, 50.000),

                                new Pose(189, 11.5)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(242), Math.toRadians(360))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(189.000, 11.5),

                                new Pose(118.875, 50.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(360), Math.toRadians(242))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(118.875, 50.0),

                                new Pose(142, 18.0)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(242), Math.toRadians(360))

                .build();
    }

    @Override
    public void init() {
        super.init();
        follower.setStartingPose(new Pose(120.000, 8.5625, Math.toRadians(180)));
        robot.setAlliance(Alliance.RED);
    }
}
