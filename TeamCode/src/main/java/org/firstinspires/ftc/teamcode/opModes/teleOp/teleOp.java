package org.firstinspires.ftc.teamcode.opModes.teleOp;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.Passthrough;

@Configurable
@TeleOp
public class teleOp extends RobotOpMode {
    private boolean slowMode = false;
    private final double slowModeMultiplier = 0.25;

    @Override
    public void init() {
        super.init();
        if (Passthrough.pose == null || Passthrough.alliance == null) {
            telemetry.addLine("Run an auto first to set the pose and alliance");
        }
        else {
            follower.setStartingPose(Passthrough.pose);
            robot.setAlliance(Passthrough.alliance);
            telemetry.addData("Alliance", Passthrough.alliance);
        }
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        double speedMultiplier = slowMode ? slowModeMultiplier : 1.0;
        double x = gamepad1.left_stick_x * speedMultiplier;
        double y = gamepad1.left_stick_y * speedMultiplier;

        double turn = -gamepad1.right_stick_x * speedMultiplier;

        if (gamepad1.leftBumperWasReleased())
            slowMode = false;
        
        follower.setTeleOpDrive(y, x, turn, false, robot.getTeleOpHeadingOffset());

        if (gamepad1.dpadLeftWasPressed()) {
            slowMode = !slowMode;
        }
    }
}
