package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class BlueBoi extends ServoImpl {

    public BlueBoi(HardwareMap hardwareMap) {
        this((ServoImpl) hardwareMap.get(Servo.class, "blueBoi"));
    }

    private BlueBoi(ServoImpl baseServo) {
        super(baseServo.getController(), baseServo.getPortNumber());
        setPosition(0.65);
    }

    public void open() { setPosition(1.0); }
    public void close() { setPosition(0.65); }

    public Command open = instant(() -> setPosition(1.0)).requiring(this);
    public Command close = instant(() -> setPosition(0.65)).requiring(this);
}