package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter with software velocity control (motor runs in RUN_WITHOUT_ENCODER):
 *
 *   power12 = kS + kV * target + kP * (target - filteredVelocity)
 *   power   = power12 * (nominalVoltage / batteryVoltage)
 *
 * Velocities are ticks/sec. kS and kV come from ShooterCharacterize
 * (kV is power per tick/sec, a multiplier).
 *
 * IMPORTANT: call shooter.update() once every loop, or the motor gets no power.
 */
public class Shooter extends DcMotorImplEx {

    // ---- tune these ----
    public static double kS = 0.08;            // from ShooterCharacterize
    public static double kV = 1.0 / 1873.0;    // from ShooterCharacterize
    public static double kP = 0.002;            // keep small: velocity only updates in 20 tick/sec steps
    public static double closeEnough = 40;     // ticks/sec window for atTarget() (velocity reads in 20 tick/sec steps)
    public static double filterAlpha = 0.15;   // 0..1, higher = less smoothing
    public static double nominalVoltage = 12.0;

    private static final double FLYWHEEL_MIN_SPEED = 800;
    private static final double FLYWHEEL_MAX_SPEED = 1500;
    private static final double VOLTAGE_READ_PERIOD_SEC = 0.2;

    private final VoltageSensor battery;
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double target = 0;
    private double filteredVelocity = 0;
    private double voltage;
    private double lastPower = 0;

    public Shooter(HardwareMap hardwareMap) {
        this((DcMotorImpl) hardwareMap.get(DcMotor.class, "shooter"),
                hardwareMap.voltageSensor.iterator().next());
    }

    private Shooter(DcMotorImpl baseMotor, VoltageSensor battery) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        this.battery = battery;
        this.voltage = battery.getVoltage();
        setDirection(DcMotorEx.Direction.REVERSE);
        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Set the target velocity in ticks/sec. 0 turns the flywheel off. */
    public void setTargetVelocity(double ticksPerSec) {
        target = Math.max(0, ticksPerSec);
    }

    /** Routes any existing shooter.setVelocity(x) calls to the software controller. */
    @Override
    public void setVelocity(double ticksPerSec) {
        setTargetVelocity(ticksPerSec);
    }

    public void off() {
        target = 0;
        lastPower = 0;
        setPower(0);
    }

    public double getTargetVelocity() {
        return target;
    }

    /** Smoothed velocity in ticks/sec (getVelocity() still returns the raw hub value). */
    public double getFilteredVelocity() {
        return filteredVelocity;
    }

    public double getLastPower() {
        return lastPower;
    }

    public boolean atTarget() {
        return target > 0 && Math.abs(target - filteredVelocity) < closeEnough;
    }

    /** Run every loop. */
    public void update() {
        double raw = getVelocity();
        filteredVelocity += filterAlpha * (raw - filteredVelocity);

        if (target <= 0) {
            lastPower = 0;
            setPower(0);
            return;
        }

        // voltage reads are slow, so only refresh a few times a second
        if (voltageTimer.seconds() > VOLTAGE_READ_PERIOD_SEC) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }

        double error = target - filteredVelocity;
        double power12 = kS + kV * target + kP * error;
        double power = power12 * (nominalVoltage / voltage);

        lastPower = Range.clip(power, 0, 1);
        setPower(lastPower);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.0202163 * velocity * velocity) - (5.99404 * velocity) + 1341.00214,
                FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    public Command setVelocityCommand(double ticks) {
        return instant(() -> setTargetVelocity(ticks)).requiring(this);
    }

    public Command off = instant(this::off).requiring(this);
}