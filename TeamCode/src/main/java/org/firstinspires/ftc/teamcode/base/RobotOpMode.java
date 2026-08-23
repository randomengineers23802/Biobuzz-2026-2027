package org.firstinspires.ftc.teamcode.base;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.control.Robot;
import org.firstinspires.ftc.teamcode.control.Passthrough;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public abstract class RobotOpMode extends OpMode {
    protected Robot robot;
    protected Follower follower;
    protected TelemetryManager panelsTelemetry;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Passthrough.pose = follower.getPose();
    }
}
