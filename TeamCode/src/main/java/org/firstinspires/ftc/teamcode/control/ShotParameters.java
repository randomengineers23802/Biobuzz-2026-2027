package org.firstinspires.ftc.teamcode.control;

public class ShotParameters {
    private final double flywheelTicks;
    private final double heading;

    public ShotParameters(double flywheelTicks, double heading) {
        this.flywheelTicks = flywheelTicks;
        this.heading = heading;
    }

    public double getFlywheelTicks() {
        return flywheelTicks;
    }

    public double getHeading() {
        return heading;
    }
}
