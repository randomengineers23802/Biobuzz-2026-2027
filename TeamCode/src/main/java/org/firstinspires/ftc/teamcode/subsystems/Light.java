package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class Light extends ServoImpl {

    public Light(HardwareMap hardwareMap) {
        this((ServoImpl) hardwareMap.get(Servo.class, "light"));
    }

    private Light(ServoImpl baseServo) {
        super(baseServo.getController(), baseServo.getPortNumber());
    }

    public void setColor(double value) {
        this.setPosition(value);
    }
    public Command setColorCommand(double value) {
        return instant(() -> setPosition(value)).requiring(this);
    }
}