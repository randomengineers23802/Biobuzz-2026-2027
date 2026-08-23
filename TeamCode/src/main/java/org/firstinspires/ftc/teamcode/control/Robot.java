package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    private Follower follower;
    private double teleOpHeadingOffset;

    public Robot(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
    }

    public void setAlliance(Alliance alliance) {
        switch (alliance) {
            case BLUE:
                teleOpHeadingOffset = Math.toRadians(0);
                break;
            case RED:
                teleOpHeadingOffset = Math.toRadians(180);
                break;
        }
        Passthrough.alliance = alliance;
    }

    public double getTeleOpHeadingOffset() {
        return teleOpHeadingOffset;
    }
}