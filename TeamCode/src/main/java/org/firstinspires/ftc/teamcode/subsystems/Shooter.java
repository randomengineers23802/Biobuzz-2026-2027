package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter extends DcMotorImplEx {
    public static double p = 50;
    public static double f = 16.05;
    private static final double NOMINAL_VOLTAGE = 13.0;
    private static final double VOLTAGE_READ_INTERVAL_MS = 250;
    private static final double F_EPSILON = 0.05;

    private static final double FLYWHEEL_MIN_SPEED = 1100;
    private static final double FLYWHEEL_MAX_SPEED = 1300;

    private final VoltageSensor voltageSensor;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage = NOMINAL_VOLTAGE;
    private double lastP = Double.NaN;
    private double lastF = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, (DcMotorImpl) hardwareMap.get(DcMotor.class, "shooter"));
    }

    private Shooter(HardwareMap hardwareMap, DcMotorImpl baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        readVoltage();
        applyPIDF();
    }

    private void readVoltage() {
        double v = voltageSensor.getVoltage();
        if (v > 1.0) voltage = v;
        voltageTimer.reset();
    }

    private void applyPIDF() {
        double compensatedF = f * (NOMINAL_VOLTAGE / voltage);
        if (p != lastP || Double.isNaN(lastF) || Math.abs(compensatedF - lastF) > F_EPSILON) {
            setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(p, 0.0, 0.0, compensatedF));
            lastP = p;
            lastF = compensatedF;
        }
    }

    public void update() {
        if (voltageTimer.milliseconds() >= VOLTAGE_READ_INTERVAL_MS) {
            readVoltage();
        }
        applyPIDF();
    }

    public double getVoltage() {
        return voltage;
    }

    public void off() {
        setPower(0);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.173861 * velocity * velocity) - (68.1358 * velocity) + 7837.50939, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }
}