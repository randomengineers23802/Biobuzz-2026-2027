package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Descore;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Stopper;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class Robot {
    private Follower follower;
    public Intake intake;
    public Transfer transfer;
    public Stopper stopper;
    public Shooter shooter;
    public Turret turret;
    public Descore descore;
    private double teleOpHeadingOffset;
    private static Pose leftHive;
    private static Pose rightHive;
    private static final double SCORE_HEIGHT = 45.5;
    private static final double PASSTHROUGH_POINT_RADIUS = 0; // distance between target point and entry plane
    private static final double TURRET_OFFSET = 6; // behind the bot, opposite of intake, guess rn
    private static final double SCORE_ANGLE = Math.toRadians(-33);
    private static final double HOOD_MIN_ANGLE = Math.toRadians(90 - 28); // ball exit angle of 62 degrees, more forward shot, hood raised highest for shot 6ft away
    private static final double HOOD_MAX_ANGLE = Math.toRadians(90 - 7); // ball exit angle of 83 degrees, almost vertical shot from close, hood lowered for shot 1ft away

    // +1 if increasing turret degrees = counterclockwise (viewed from above), -1 if clockwise.
    // Last year's turret was clockwise-positive, so start with -1
    private static final double TURRET_DIRECTION = -1;
    private static final double TURRET_ZERO_ANGLE = 0; // radians, where turret 0° points relative to the robot's front

    // Last turret angle sent to the servos (degrees, -10 to 370). Stands in for the encoder.
    private double lastCommandedDeg = Turret.TURRET_START_DEGREES;

    public Robot(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        stopper = new Stopper(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        descore = new Descore(hardwareMap);
    }

    public void updateTurretAndShooter() {
        Pose currentPose = follower.pose();
        Pose target = getHiveTargetPose(currentPose);
        Pose offset = turretPoseOffset(currentPose);
        Pose turretFieldPose = new Pose(currentPose.x() + offset.x(), currentPose.y() + offset.y());

        Vector2D robotToHiveVector = robotToHiveVector(turretFieldPose, target);
        double g = 32.174 * 12;
        double x = robotToHiveVector.magnitude() - PASSTHROUGH_POINT_RADIUS;
        double y = SCORE_HEIGHT;
        double a = SCORE_ANGLE;
        if (x < 1) return; // too close / degenerate, hold last commands

        // Initial launch components
        double hoodAngle = Range.clip(Math.atan(2 * y / x - Math.tan(a)), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        double denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y);
        if (denom <= 0) return;
        double flywheelSpeed = Math.sqrt(g * x * x / denom);

        Vector2D robotVelocity = follower.velocity().toVector2D();
        double coordinateTheta = robotVelocity.theta() - robotToHiveVector.theta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.magnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.magnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        // Recalculate launch components using velocity
        hoodAngle = Range.clip(Math.atan(vz / nvr), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE);
        denom = 2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y);
        if (denom <= 0) return;
        flywheelSpeed = Math.sqrt(g * ndr * ndr / denom);

        double turretVelCompOffset = Math.atan2(perpendicularComponent, ivr);

        // Field-relative angle to the hive with the velocity lead applied
        double fieldTargetRad = robotToHiveVector.theta() - turretVelCompOffset;

        // Convert to robot-relative by removing the robot heading and turret zero offset
        double robotRelativeTargetRad = fieldTargetRad - currentPose.heading() - TURRET_ZERO_ANGLE;

        // Convert to the turret's own rotation direction
        double turretTargetRad = TURRET_DIRECTION * robotRelativeTargetRad;

        // Shortest path from the last commanded angle, folded back into [-10, 370]
        double targetDeg = Math.toDegrees(turretTargetRad);
        double bestTargetDeg = lastCommandedDeg + Turret.wrap180(targetDeg - lastCommandedDeg);
        if (bestTargetDeg > Turret.TURRET_MAX_DEGREES)
            bestTargetDeg -= 360;
        else if (bestTargetDeg < Turret.TURRET_MIN_DEGREES)
            bestTargetDeg += 360;
        lastCommandedDeg = bestTargetDeg;

        shooter.setVelocity(Shooter.getFlywheelTicksFromVelocity(flywheelSpeed));
        turret.setTurretPosition(Turret.degreesToServo(bestTargetDeg));
        turret.setHoodPosition(Turret.getHoodServoPositionFromDegrees(Math.toDegrees(hoodAngle)));
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                teleOpHeadingOffset = Math.toRadians(0);
                leftHive = new Pose(84, 59);
                rightHive = new Pose(84, 85);
                break;
            case RED:
                teleOpHeadingOffset = Math.toRadians(180);
                leftHive = new Pose(60, 85);
                rightHive = new Pose(60, 59);
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

    private static Pose getHiveTargetPose(Pose currentPose) {
        if (Passthrough.alliance == Alliance.BLUE && currentPose.y() >= 72)
            return rightHive;
        if (Passthrough.alliance == Alliance.BLUE && currentPose.y() < 72)
            return leftHive;
        if (Passthrough.alliance == Alliance.RED && currentPose.y() < 72)
            return rightHive;
        else
            return leftHive;
    }

    private static Vector2D robotToHiveVector(Pose currentPose, Pose target) {
        double dx = target.x() - currentPose.x();
        double dy = target.y() - currentPose.y();
        return Vector2D.cartesian(dx, dy);
    }
}