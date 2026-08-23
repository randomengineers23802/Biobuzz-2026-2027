package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Belt extends DcMotorImplEx {

    public Belt(HardwareMap hardwareMap) {
        this((DcMotorImplEx) hardwareMap.get(DcMotorEx.class, "belt"));
    }

    private Belt(DcMotorImplEx baseMotor) {
        super(baseMotor.getController(), baseMotor.getPortNumber(), baseMotor.getDirection());
        PIDFCoefficients beltPIDF = new PIDFCoefficients(0.0, 0.0, 0.0, 12.7);
        setDirection(DcMotor.Direction.FORWARD);
        setMode(RunMode.RUN_USING_ENCODER);
        setPIDFCoefficients(RunMode.RUN_USING_ENCODER, beltPIDF);
    }

    public void onShoot() { setVelocity(1600); }
    public void onIntake() { setVelocity(2600); }
    public void off() { setPower(0.0); }
    public Command onShoot = instant(() -> setVelocity(1600)).requiring(this);
    public Command onFastShoot = instant(() -> setVelocity(2200)).requiring(this);
    public Command onIntake = instant(() -> setVelocity(2600)).requiring(this);
    public Command off = instant(() -> setVelocity(0)).requiring(this);
}