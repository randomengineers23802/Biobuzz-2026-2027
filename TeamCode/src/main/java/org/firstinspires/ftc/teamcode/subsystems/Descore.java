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
        setPosition(0.53);  //idk rn for all the servo values
    }
    public void down() {setPosition(0.97); }
    public void up() {setPosition(0.53); }

    public Command down = instant(() -> setPosition(0.97)).requiring(this);
    public Command up = instant(() -> setPosition(0.53)).requiring(this);
}
