package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class Descore extends ServoImpl {
    public Descore(HardwareMap hardwareMap) {
        this((ServoImpl) hardwareMap.get(Servo.class, "descore"));
    }

    private Descore(ServoImpl baseServo) {
        super(baseServo.getController(), baseServo.getPortNumber());
    }
    public void down() {setPosition(0.97); }
    public void up() {setPosition(0.53); }

    public Command down = instant(this::down).requiring(this);
    public Command up = instant(this::up).requiring(this);
}
