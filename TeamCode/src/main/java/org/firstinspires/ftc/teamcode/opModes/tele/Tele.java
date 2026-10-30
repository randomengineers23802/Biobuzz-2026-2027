package org.firstinspires.ftc.teamcode.opModes.tele;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.Passthrough;

@TeleOp
public class Tele extends RobotOpMode {
    private boolean slowMode;
    private final double SLOW_MODE_MULTIPLIER = 0.25;

    private static final double REVERSE_DURATION_MS = 1000;
    private final ElapsedTime reverseTimer = new ElapsedTime();
    private boolean reversing = false;
    private boolean fullWasTrue = false;
    private boolean descoreDeployed = false;

    @Override
    public void init() {
        super.init();
        if (Passthrough.pose == null || Passthrough.alliance == null) {
            telemetry.addLine("Run an auto first to set the pose and alliance");
            follower.setPose(new Pose(9, 9, 0));
            robot.setAlliance(Alliance.BLUE);
        }
        else {
            follower.setPose(Passthrough.pose);
            robot.setAlliance(Passthrough.alliance);
            telemetry.addData("Alliance", Passthrough.alliance);
        }
        gamepad1.setTriggerThreshold(0.2f);
        telemetry.update();
    }

    @Override
    public void start() {
        robot.descore.up();
        robot.stopper.close();
    }

    @Override
    public void loop() {
        super.loop();

        double speedMultiplier = slowMode ? SLOW_MODE_MULTIPLIER : 1.0;
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                follower.pose().heading(),
                robot.getTeleOpHeadingOffset()
        );
        follower.manual(powers);

        if (gamepad1.leftBumperWasPressed())
            slowMode = !slowMode;

        boolean full = robot.transfer.full();
        if (full && !fullWasTrue) {
            reversing = true;
            reverseTimer.reset();
        }
        fullWasTrue = full;

        if (reversing && reverseTimer.milliseconds() >= REVERSE_DURATION_MS)
            reversing = false;

        if (reversing) {
            robot.transfer.on();
            robot.intake.reverse();
        }
        else if (gamepad1.left_trigger > 0.2) {
            robot.transfer.on();
            robot.intake.on();
        }
        else {
            robot.transfer.off();
            robot.intake.off();
        }

        if (gamepad1.rightTriggerWasPressed())
            robot.stopper.open();
        else
            robot.stopper.close();

        if (gamepad1.rightBumperWasPressed()) {
            if (descoreDeployed) {
                robot.descore.up();
                descoreDeployed = false;
            }
            else {
                robot.descore.down();
                descoreDeployed = true;
            }
        }

        telemetry.addData("pollen3", robot.transfer.pollen3State); //check this again
        telemetry.update();
    }
}