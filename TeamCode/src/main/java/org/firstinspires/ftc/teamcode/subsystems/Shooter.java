package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

public class Shooter extends DcMotorImplEx {
    PIDFCoefficients shooterPIDF = new PIDFCoefficients(115.0, 0.0, 0.0, 13.0);
    private static final double FLYWHEEL_MIN_SPEED = 940;
    private static final double FLYWHEEL_MAX_SPEED = 1440;

    public Shooter(HardwareMap hardwareMap) {
        this((DcMotorImpl) hardwareMap.get(DcMotor.class, "shooter"));
    }

    private Shooter(DcMotorImpl baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        setDirection(DcMotorEx.Direction.REVERSE);
        setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterPIDF);

    }

    public void setVelocity(double ticks) {
        setVelocity(ticks);
    }

    public void off() {
        setPower(0);
    }

    public static double getFlywheelTicksFromVelocity(double velocity) {
        return Range.clip((0.0202163 * velocity * velocity) - (5.99404 * velocity) + 1341.00214, FLYWHEEL_MIN_SPEED, FLYWHEEL_MAX_SPEED);
    }

    public Command setVelocityCommand(double ticks) {
        return instant(() -> setVelocity(ticks)).requiring(this);
    }

    public Command off = instant(this::off).requiring(this);
}
