package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lights {
    private Servo light;
    public Lights(HardwareMap hardwareMap) {
        light = hardwareMap.get(Servo.class, "light");
        light.setPosition(0.277); //red
    }


    public void update() {
        switch (Transfer.getPollenCount()) {
            case 4:
                light.setPosition(0.515); //dark green
                break;
            case 3:
                light.setPosition(0.44); //green
                break;
            case 2:
                light.setPosition(0.36); //orange yellow
                break;
            case 1:
                light.setPosition(0.32); //orange
                break;
            case 0:
                light.setPosition(0.277); //red
                break;
        }
    }
}
