package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends DcMotorImpl {

    public Intake(HardwareMap hardwareMap) {
        this((DcMotorImpl) hardwareMap.get(DcMotor.class, "intake"));
    }

    private Intake(DcMotorImpl baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber());
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        setDirection(DcMotor.Direction.REVERSE);
    }

    public void on() {
        setPower(1.0);
    }

    public void off() {
        setPower(0.1);
    }
    public Command on = instant(() -> setPower(1.0)).requiring(this);
    public Command off = instant(() -> setPower(0.2)).requiring(this);
}