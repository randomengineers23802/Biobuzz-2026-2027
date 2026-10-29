package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {

    private static final double DEBOUNCE_MS = 100;

    public DcMotor motor;
    private DigitalChannel pollen1;
    private DigitalChannel pollen2;
    private DigitalChannel pollen3;
    private DigitalChannel pollen4;

    private boolean pollen1RawState;
    private boolean pollen2RawState;
    private boolean pollen3RawState;
    private boolean pollen4RawState;

    public boolean pollen1State;
    public boolean pollen2State;
    public boolean pollen3State;
    public boolean pollen4State;

    private final ElapsedTime pollen1Timer = new ElapsedTime();
    private final ElapsedTime pollen2Timer = new ElapsedTime();
    private final ElapsedTime pollen3Timer = new ElapsedTime();
    private final ElapsedTime pollen4Timer = new ElapsedTime();

    public Transfer(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "transfer");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollen1 = hardwareMap.get(DigitalChannel.class, "pollen1");
        pollen1.setMode(DigitalChannel.Mode.INPUT);
        pollen2 = hardwareMap.get(DigitalChannel.class, "pollen2");
        pollen2.setMode(DigitalChannel.Mode.INPUT);
        pollen3 = hardwareMap.get(DigitalChannel.class, "pollen3");
        pollen3.setMode(DigitalChannel.Mode.INPUT);
        pollen4 = hardwareMap.get(DigitalChannel.class, "pollen4");
        pollen4.setMode(DigitalChannel.Mode.INPUT);
    }

    public void updatePollenState() {
        pollen1RawState = pollen1.getState();
        pollen2RawState = pollen2.getState();
        pollen3RawState = pollen3.getState();
        pollen4RawState = pollen4.getState();

        pollen1State = debounce(pollen1RawState, pollen1Timer);
        pollen2State = debounce(pollen2RawState, pollen2Timer);
        pollen3State = debounce(pollen3RawState, pollen3Timer);
        pollen4State = debounce(pollen4RawState, pollen4Timer);

        pollen3State = pollen3State || pollen4State;
        pollen2State = pollen2State || pollen3State;
        pollen1State = pollen1State || pollen2State;
    }

    private boolean debounce(boolean rawState, ElapsedTime timer) {
        if (!rawState) {
            timer.reset();
            return false;
        }
        return timer.milliseconds() >= DEBOUNCE_MS;
    }

    public void on() {
        motor.setPower(1.0);
    }

    public void off() {
        motor.setPower(0.0);
    }

    public Command on = instant(() -> motor.setPower(1.0)).requiring(this);
    public Command off = instant(() -> motor.setPower(0.0)).requiring(this);
}