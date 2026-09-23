package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Robot {
    private Follower follower;
    private double teleOpHeadingOffset;
    private final PIDF turretPIDF = new PIDF(1.2, 0.0, 0.1, 0.02, Math.toRadians(1.0), -1.0, 1.0, PIDF.Mode.HEADING);
    private static final double FLYWHEEL_MIN_SPEED = 940;
    private static final double FLYWHEEL_MAX_SPEED = 1440;
    private static final double SCORE_HEIGHT = 45.5;
    private static final double PASSTHROUGH_POINT_RADIUS = 0; //distance between target point and entry plane
    private static final double TURRET_OFFSET = 6;//behind the bot, opposite of intake, guess rn
    private static final double SCORE_ANGLE = Math.toRadians(-33);
    private static final double HOOD_MIN_ANGLE = Math.toRadians(90 - 28); //ball exit angle of 62 degrees, more forward shot, hood raised highest for shot 6ft away
    private static final double HOOD_MAX_ANGLE = Math.toRadians(90 - 7); //ball exit angle of 83 degrees, almost vertical shot from close, hood lowered for shot 1ft away

    public Robot(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
    }

    public ShotParameters updateShotParameters(Pose target) {
        Pose currentPose = follower.pose();
        Pose offset = turretPoseOffset(currentPose);
        Pose turretFieldPose = new Pose(currentPose.x() + offset.x(), currentPose.y() + offset.y());

        Vector2D robotToGoalVector = robotToHiveVector(turretFieldPose, target);
        double g = 32.174 * 12;
        double x = robotToGoalVector.magnitude() - PASSTHROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;

        //Calculate initial launch components
        double hoodAngle = Range.clip(Math.atan(2 * y / x - Math.tan(a)), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        Vector2D robotVelocity = follower.velocity().toVector2D();
        double coordinateTheta = robotVelocity.theta() - robotToGoalVector.theta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.magnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.magnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        //Recalculate launch components using velocity
        hoodAngle = Range.clip(Math.atan(vz / nvr), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
        
        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);

        double heading = robotToGoalVector.theta() - turretVelCompOffset + Math.PI;
        double flywheelTicks = getFlywheelTicksFromVelocity(flywheelSpeed);
        double hoodServoPos = getHoodServoPositionFromDegrees(Math.toDegrees(hoodAngle));

        turretPIDF.setTarget(heading);
        double aimPower = turretPIDF.calculate(currentPose.heading());

        return new ShotParameters(flywheelTicks, aimPower, hoodServoPos);
        //When we use axons to aim turret will have to return between 0 and 1 for aimPower
        //Won't need pidf just method to convert heading angle to servo pos
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                teleOpHeadingOffset = Math.toRadians(0);
                break;
            case RED:
                teleOpHeadingOffset = Math.toRadians(180);
                break;
        }
        Passthrough.alliance = alliance;
    }

    public double getTeleOpHeadingOffset() {
        return teleOpHeadingOffset;
    }

    private static Pose turretPoseOffset(Pose centerPose) {
        return new Pose(Math.cos(centerPose.heading() + Math.PI) * TURRET_OFFSET, Math.sin(centerPose.heading() + Math.PI) * TURRET_OFFSET);
    }

    private static Vector2D robotToHiveVector(Pose currentPose, Pose target) {
        double dx = target.x() - currentPose.x();
        double dy = target.y() - currentPose.y();
        return Vector2D.cartesian(dx, dy);
    }

    private static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.0202163 * velocity * velocity) - (5.99404 * velocity) + 1341.00214, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    private static double getHoodServoPositionFromDegrees(double degrees) {
        return Range.clip(0.00483871 * degrees - 0.215161, 0.05, 0.11);
    }

//    // Calibrate the exact electrical ceiling of your Hub/ADC
//    final double VOLTAGE_MIN = 0.00; // Expected voltage at 0 degrees
//    final double VOLTAGE_MAX = 3.22; // Typical max voltage read at 350 degrees
//
//    double currentVoltage = axonFeedback.getVoltage(); //get actual analog input
//
//    // Convert directly to a clean 0.0 - 1.0 range
//    double servoPos = (currentVoltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN); //can remove voltage min from  equation if it truly hits 0 or close enough
}