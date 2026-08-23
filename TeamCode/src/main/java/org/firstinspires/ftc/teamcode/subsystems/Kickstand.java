package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kickstand {
    public Servo kickstand1;
    public Servo kickstand2;
    public Servo kickstand3;

    public Kickstand(HardwareMap hardwareMap) {
        kickstand1 = hardwareMap.get(Servo.class, "kickstand1");
        kickstand2 = hardwareMap.get(Servo.class, "kickstand2");
        kickstand3 = hardwareMap.get(Servo.class, "kickstand3");
    }

    public void raise() {
        kickstand1.setPosition(0.16);
        kickstand2.setPosition(0.86);
        kickstand3.setPosition(0.16);
    }

    public void lower() {
        kickstand1.setPosition(0.58);
        kickstand2.setPosition(0.44);
        kickstand3.setPosition(0.58);
    }

    public Command raise = instant(() -> {
        kickstand1.setPosition(0.16);
        kickstand2.setPosition(0.86);
        kickstand3.setPosition(0.16);
    }).requiring(this);

    public Command lower = instant(() -> {
        kickstand1.setPosition(0.58);
        kickstand2.setPosition(0.44);
        kickstand3.setPosition(0.58);
    }).requiring(this);
}
