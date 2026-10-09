package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("motorFL");
        c.frontRightName.set("motorFR");
        c.backLeftName.set("motorBL");
        c.backRightName.set("motorBR");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("Odometry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.6628758888544999);
        c.yPodOffset.set(-0.25081294728076364);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.705095110634864);
                Controller secondaryTranslationalForward = Controller.proportional(0.2605138028912359);
                Controller primaryTranslationalLateral = Controller.proportional(0.2785382434106006);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10291243826127922);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009201203376034542));
                c.brake.set(Controller.proportionalFeedforward(0.00782102286962936));

                c.headingFeedback.set(Controller.proportional(3.7250398840307217));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.055062743787985625, 0.006443969812158143));

                c.linearBrakeCoefficients.set(Matrix.diag(0.010817182310239752, 0.05551090096703573));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0020361481709691817, 0.0017531896499794995));

                c.maxAchievableForwardVelocity.set(105.97600810144908);
                c.maxAchievableStrafeVelocity.set(83.37618791929656);
                c.naturalForwardDeceleration.set(1649.021814003655);
                c.naturalStrafeDeceleration.set(51.389586941690325);
            }
    );
}