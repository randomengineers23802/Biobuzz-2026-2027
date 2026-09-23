package org.firstinspires.ftc.teamcode.opModes.tele;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.Passthrough;

@TeleOp
public class Tele extends RobotOpMode {
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    @Override
    public void init() {
        super.init();
        if (Passthrough.pose == null || Passthrough.alliance == null) {
            telemetry.addLine("Run an auto first to set the pose and alliance");
            follower.setPose(new Pose(0, 0, 0));
        }
        else {
            follower.setPose(Passthrough.pose);
            robot.setAlliance(Passthrough.alliance);
            telemetry.addData("Alliance", Passthrough.alliance);
        }
        telemetry.update();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
        double speedMultiplier = slowMode ? slowModeMultiplier : 1.0;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * speedMultiplier,
                gamepad1.left_stick_x * speedMultiplier,
                gamepad1.right_stick_x * speedMultiplier,
                follower.pose().heading(),
                robot.getTeleOpHeadingOffset()
        );
        follower.manual(powers);

        if (gamepad1.dpadLeftWasPressed()) {
            slowMode = !slowMode;
        }
    }
}
