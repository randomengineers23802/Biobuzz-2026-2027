package org.firstinspires.ftc.teamcode.control;

public class ShotParameters {
    private final double flywheelTicks;
    private final double turretHeading;
    private final double hoodAngle;

    public ShotParameters(double flywheelTicks, double turretHeading, double hoodAngle) {
        this.flywheelTicks = flywheelTicks;
        this.turretHeading = turretHeading;
        this.hoodAngle = hoodAngle;
    }

    public double getFlywheelTicks() {
        return flywheelTicks;
    }

    public double getHeading() {
        return turretHeading;
    }

    public double getHoodAngle() {
        return hoodAngle;
    }
}
