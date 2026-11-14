package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDF {

    public enum Mode {
        GENERIC,
        HEADING
    }

    private final double kP;
    private final double kI;
    private final double kD;
    private final double f;
    private final double deadzone;
    private final double minOutput;
    private final double maxOutput;
    private final Mode mode;
    private double target = 0;
    private double integral = 0;
    private double lastPosition = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDF(double kP, double kI, double kD, double f, Mode mode, double deadzone, double minOutput, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.f = f;
        this.deadzone = deadzone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.mode = mode;
    }

    public PIDF(double kP, double kI, double kD, double f, Mode mode) {
        this(kP, kI, kD, f, mode, 0, -1, 1);
    }

    public PIDF(double kP, double kI, double kD, double f, Mode mode, double deadzone) {
        this(kP, kI, kD, f, mode, deadzone, -1, 1);
    }

    public PIDF(double kP, double kI, double kD, double f, Mode mode, double minOutput, double maxOutput) {
        this(kP, kI, kD, f, mode, 0, minOutput, maxOutput);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void reset() {
        integral = 0;
        lastPosition = 0;
        timer.reset();
    }

    public double calculate(double currentPosition) {
        double dt = timer.seconds();
        timer.reset();

        double error = target - currentPosition;
        if (mode == Mode.HEADING)
            error = Angle.normalizeSigned(error);

        if (Math.abs(error) <= deadzone) {
            lastPosition = currentPosition;
            return 0.0;
        }

        double derivative = 0;
        if (dt > 0.0001) {
            if (mode == Mode.HEADING)
                derivative = -Angle.normalizeSigned(currentPosition - lastPosition) / dt;
            else
                derivative = -(currentPosition - lastPosition) / dt;
            integral += error * dt;
        }

        lastPosition = currentPosition;

        double feedForward = Math.signum(error);

        double output = (error * kP) + (integral * kI) + (derivative * kD) + (feedForward * f);

        return Range.clip(output, minOutput, maxOutput);
    }
}