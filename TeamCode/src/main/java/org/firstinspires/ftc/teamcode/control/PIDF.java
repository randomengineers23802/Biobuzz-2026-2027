package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {

    public enum Mode {
        GENERIC,
        HEADING
    }

    private final double p;
    private final double i;
    private final double d;
    private final double f;
    private final double deadzone;
    private final double minOutput;
    private final double maxOutput;
    private final Mode mode;
    private double target = 0;
    private double integral = 0;
    private double lastPosition = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDF(double p, double i, double d, double f, double deadzone, double minOutput, double maxOutput, Mode mode) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.deadzone = deadzone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.mode = mode;
    }

    public PIDF(double p, double i, double d, double f, Mode mode) {
        this(p, i, d, f, 0.0, -1.0, 1.0, mode);
    }

    public PIDF(double p, double i, double d, double f, double deadzone, Mode mode) {
        this(p, i, d, f, deadzone, -1.0, 1.0, mode);
    }

    public PIDF(double p, double i, double d, double f, double minOutput, double maxOutput, Mode mode) {
        this(p, i, d, f, 0.0, minOutput, maxOutput, mode);
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
        if (mode == Mode.HEADING) {
            error = MathFunctions.normalizeAngleSigned(error);
        }

        if (Math.abs(error) <= deadzone) {
            lastPosition = currentPosition;
            return 0.0;
        }

        double derivative = 0.0;
        if (dt > 0.0001) {
            if (mode == Mode.HEADING) {
                derivative = -MathFunctions.normalizeAngleSigned(currentPosition - lastPosition) / dt;
            } else {
                derivative = -(currentPosition - lastPosition) / dt;
            }
            integral += error * dt;
        }

        lastPosition = currentPosition;

        double feedForward = Math.signum(error);

        double output = (error * p) + (integral * i) + (derivative * d) + (feedForward * f);
        return MathFunctions.clamp(output, minOutput, maxOutput);
    }
}