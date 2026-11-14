package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public ServoImplEx axonTurret1;
    public ServoImplEx axonTurret2;
    public Servo hood;
    private static final double SPAN_MIN_DEGREES = -11.6665;
    private static final double SPAN_MAX_DEGREES = 371.6665;
    public static final double TURRET_MIN_DEGREES = 90;
    public static final double TURRET_MAX_DEGREES = 270;
    public static final double TURRET_START_DEGREES = 180;

    public Turret(HardwareMap hardwareMap) {
        axonTurret1 = hardwareMap.get(ServoImplEx.class, "axonTurret1");
        axonTurret2 = hardwareMap.get(ServoImplEx.class, "axonTurret2");
        PwmControl.PwmRange servoRange = new PwmControl.PwmRange(500, 2500);
        axonTurret1.setPwmRange(servoRange);
        axonTurret2.setPwmRange(servoRange);
        hood = hardwareMap.get(Servo.class, "hood");
    }

    public static double normalizeToTurretRange(double degrees) {
        double d = ((degrees % 360.0) + 360.0) % 360.0;

        double[] candidates = {d - 360.0, d, d + 360.0};
        for (double c : candidates) {
            if (c >= TURRET_MIN_DEGREES && c <= TURRET_MAX_DEGREES) {
                return c;
            }
        }

        double toMin = Math.abs(wrap180(d - TURRET_MIN_DEGREES));
        double toMax = Math.abs(wrap180(d - TURRET_MAX_DEGREES));
        return toMin <= toMax ? TURRET_MIN_DEGREES : TURRET_MAX_DEGREES;
    }

    public static double degreesToServo(double degrees) {
        double reachable = normalizeToTurretRange(degrees);
        double clamped = Math.max(TURRET_MIN_DEGREES, Math.min(TURRET_MAX_DEGREES, reachable));
        double normalized = (clamped - SPAN_MIN_DEGREES) / (SPAN_MAX_DEGREES - SPAN_MIN_DEGREES);
        return Range.clip(1.0 - normalized, 0.0, 1.0);
    }

    public void setTurretServoPosition(double position) {
        axonTurret1.setPosition(position);
        axonTurret2.setPosition(position);
    }

    public static double wrap180(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }
    
    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public static double getHoodServoPositionFromDegrees(double degrees) {
        return Range.clip(-0.0222222 * degrees + 2.04333, 0.21, 0.55);
    }

    public void turretStart() {
        setTurretServoPosition(degreesToServo(TURRET_START_DEGREES));
    }
}