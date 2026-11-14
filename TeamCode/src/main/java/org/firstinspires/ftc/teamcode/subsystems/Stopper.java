package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

public class Stopper extends ServoImpl {
    public Stopper(HardwareMap hardwareMap) {
        this((ServoImpl) hardwareMap.get(Servo.class, "stopper"));
    }

    private Stopper(ServoImpl baseServo) {
        super(baseServo.getController(), baseServo.getPortNumber());
    }
    public void open() {setPosition(0.97); }
    public void close() {setPosition(0.53); }

    public Command open = instant(this::open);
    public Command close = instant(this::close);
    }
