package org.firstinspires.ftc.teamcode.control;

public enum Target {
    GOAL1,
    GOAL2;
    public Target toggle() {
        return (this == GOAL1) ? GOAL2 : GOAL1;
    }
}