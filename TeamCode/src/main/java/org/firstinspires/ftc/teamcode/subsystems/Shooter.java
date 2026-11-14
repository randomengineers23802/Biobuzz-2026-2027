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
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Shooter extends DcMotorImplEx {
    public static double p = 100;
    public static double f = 20.5;
    PIDFCoefficients shooterPIDF = new PIDFCoefficients(p, 0.0, 0.0, f);
    private static final double FLYWHEEL_MIN_SPEED = 1140;
    private static final double FLYWHEEL_MAX_SPEED = 1300;

    public Shooter(HardwareMap hardwareMap) {
        this((DcMotorImpl) hardwareMap.get(DcMotor.class, "shooter"));
    }

    private Shooter(DcMotorImpl baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);

    }

    public void off() {
        setPower(0);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.173861 * velocity * velocity) - (68.1358 * velocity) + 7837.50939, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    public Command setVelocityCommand(double ticks) {
        return instant(() -> setVelocity(ticks)).requiring(this);
    }

    public Command off = instant(this::off).requiring(this);
}