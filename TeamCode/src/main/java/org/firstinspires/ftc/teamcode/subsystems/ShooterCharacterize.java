package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Steps the shooter through a set of open-loop powers, records the settled velocity at each,
 * and fits   power12 = kS + kV * velocity   by linear regression.
 *
 * power12 is the power converted to a 12V equivalent (power * measuredVoltage / 12),
 * which matches how Shooter.java scales its output by 12 / batteryVoltage.
 *
 * Run with the counters connected, no balls loaded, and a fresh battery.
 * Press play and wait about 30 seconds. Copy kS and kV into Shooter.java.
 */
@TeleOp(name = "Shooter Characterize", group = "Test")
public class ShooterCharacterize extends OpMode {

    private static final double[] POWERS = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    private static final double SETTLE_SEC = 2.5;
    private static final double SAMPLE_SEC = 1.0;
    private static final double MIN_VALID_VELOCITY = 50; // ignore points where it barely spun

    private DcMotorEx motor;
    private VoltageSensor battery;
    private final ElapsedTime timer = new ElapsedTime();

    private int step = 0;
    private boolean done = false;

    private double velSum = 0, voltSum = 0;
    private int count = 0;

    private final double[] velocity = new double[POWERS.length];
    private final double[] power12 = new double[POWERS.length];

    private double kS, kV, r2;
    private int pointsUsed;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("Press play. Runs about 30 sec through " + POWERS.length + " power steps.");
        telemetry.update();
    }

    @Override
    public void start() {
        motor.setPower(POWERS[0]);
        timer.reset();
    }

    @Override
    public void loop() {
        if (!done) {
            double t = timer.seconds();

            if (t > SETTLE_SEC) {
                velSum += motor.getVelocity();
                voltSum += battery.getVoltage();
                count++;
            }

            if (t > SETTLE_SEC + SAMPLE_SEC && count > 0) {
                double avgVel = velSum / count;
                double avgVolt = voltSum / count;
                velocity[step] = avgVel;
                power12[step] = POWERS[step] * avgVolt / 12.0;

                step++;
                velSum = 0;
                voltSum = 0;
                count = 0;

                if (step >= POWERS.length) {
                    motor.setPower(0);
                    done = true;
                    fit();
                } else {
                    motor.setPower(POWERS[step]);
                    timer.reset();
                }
            }
        }

        if (!done) {
            telemetry.addData("Step", "%d / %d (power %.2f)", step + 1, POWERS.length, POWERS[step]);
            telemetry.addData("Time in step", "%.1f s", timer.seconds());
            telemetry.addData("Velocity now", "%.0f ticks/s", motor.getVelocity());
        } else {
            telemetry.addLine("DONE");
            if (pointsUsed >= 3) {
                telemetry.addData("kS", "%.4f", kS);
                telemetry.addData("kV (power per tick/s)", "%.6f", kV);
                telemetry.addData("1 / kV", "%.0f ticks/s", 1.0 / kV);
                telemetry.addData("R^2", "%.4f", r2);
            } else {
                telemetry.addLine("Not enough valid points to fit. Check the motor name and wiring.");
            }
        }

        telemetry.addLine();
        for (int i = 0; i < POWERS.length; i++) {
            if (i < step || (done && i < POWERS.length)) {
                telemetry.addLine(String.format("p=%.2f  v=%.0f  p12=%.3f", POWERS[i], velocity[i], power12[i]));
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }

    private void fit() {
        double sx = 0, sy = 0, sxy = 0, sxx = 0;
        int n = 0;
        for (int i = 0; i < POWERS.length; i++) {
            if (velocity[i] < MIN_VALID_VELOCITY) continue;
            sx += velocity[i];
            sy += power12[i];
            sxy += velocity[i] * power12[i];
            sxx += velocity[i] * velocity[i];
            n++;
        }
        pointsUsed = n;
        if (n < 3) return;

        double denom = n * sxx - sx * sx;
        kV = (n * sxy - sx * sy) / denom;
        kS = (sy - kV * sx) / n;

        double meanY = sy / n;
        double ssTot = 0, ssRes = 0;
        for (int i = 0; i < POWERS.length; i++) {
            if (velocity[i] < MIN_VALID_VELOCITY) continue;
            double predicted = kS + kV * velocity[i];
            ssRes += (power12[i] - predicted) * (power12[i] - predicted);
            ssTot += (power12[i] - meanY) * (power12[i] - meanY);
        }
        r2 = ssTot == 0 ? 1 : 1 - ssRes / ssTot;
    }
}
