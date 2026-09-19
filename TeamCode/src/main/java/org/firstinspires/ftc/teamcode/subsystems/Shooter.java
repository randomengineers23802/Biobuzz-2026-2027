package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Shooter {
    public DcMotorEx shooterL;
    public DcMotorEx shooterR;
    PIDFCoefficients shooterLPIDF = new PIDFCoefficients(115.0, 0.0, 0.0, 13.0);
    PIDFCoefficients shooterRPIDF = new PIDFCoefficients(115.0, 0.0, 0.0, 12.4);

    public Shooter(HardwareMap hardwareMap) {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterL");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterR");
        shooterL.setDirection(DcMotorEx.Direction.REVERSE);
        shooterR.setDirection(DcMotorEx.Direction.FORWARD);
        shooterL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterL.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterLPIDF);
        shooterR.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, shooterRPIDF);
    }

    public void setVelocity(double ticks) {
        shooterL.setVelocity(ticks);
        shooterR.setVelocity(ticks);
    }

    public void off() {
        shooterL.setPower(0);
        shooterR.setPower(0);
    }

    public Command setVelocityCommand(double ticks) {
        return instant(() -> setVelocity(ticks)).requiring(this);
    }

    public Command off = instant(this::off).requiring(this);
}
