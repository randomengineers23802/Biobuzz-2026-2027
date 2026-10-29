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
        c.xPodOffset.set(0.078125);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.288354667430495);
                Controller secondaryTranslationalForward = Controller.proportional(0.10653934463694953);
                Controller primaryTranslationalLateral = Controller.proportional(0.3924239392100688);
                Controller secondaryTranslationalLateral = Controller.proportional(0.14499015977734572);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009424209142890673));
                c.brake.set(Controller.proportionalFeedforward(0.008010577771457071));

                c.headingFeedback.set(Controller.proportional(4.693277988469963));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05613787699102601, 0.006657880040576454));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11357385680272561, 0.08217103617267803));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013576666933053255, 0.001434247342098266));

                c.maxAchievableForwardVelocity.set(95.01476686156569);
                c.maxAchievableStrafeVelocity.set(79.09711815743852);
                c.naturalForwardDeceleration.set(27.60721698413197);
                c.naturalStrafeDeceleration.set(60.72065122261073);
            }
    );
}

