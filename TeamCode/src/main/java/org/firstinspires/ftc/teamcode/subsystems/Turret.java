package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public Servo axonTurret1;
    public Servo axonTurret2;
    public Servo hood;

    public static final double TURRET_MIN_DEGREES = -10;
    public static final double TURRET_MAX_DEGREES = 370;
    public static final double TURRET_START_DEGREES = 180;

    public Turret(HardwareMap hardwareMap) {
        axonTurret1 = hardwareMap.get(Servo.class, "axonTurret1");
        axonTurret2 = hardwareMap.get(Servo.class, "axonTurret2");
        hood = hardwareMap.get(Servo.class, "hood");
        setTurretPosition(degreesToServo(TURRET_START_DEGREES));
    }

    public static double degreesToServo(double degrees) {
        double clamped = Math.max(TURRET_MIN_DEGREES, Math.min(TURRET_MAX_DEGREES, degrees));
        return (clamped - TURRET_MIN_DEGREES) / (TURRET_MAX_DEGREES - TURRET_MIN_DEGREES);
    }

    public void setTurretPosition(double position) { // 0.0 to 1.0
        axonTurret1.setPosition(position);
        axonTurret2.setPosition(position);
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public static double wrap180(double deg) {
        return ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    public static double getHoodServoPositionFromDegrees(double degrees) {
        return Range.clip(0.00483871 * degrees - 0.215161, 0.05, 0.11);
    }
}