package org.firstinspires.ftc.teamcode.opModes.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.Passthrough;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp
public class Tele extends RobotOpMode {
    private boolean slowMode;
    private final double SLOW_MODE_MULTIPLIER = 0.25;

    private static final double REVERSE_DURATION_MS = 1000;
    private final ElapsedTime reverseTimer = new ElapsedTime();
    private boolean reversing = false;
    private boolean fullWasTrue = false;
    private boolean descoreDeployed = false;

    private static final Pose DEFAULT_POSE = new Pose(10, 8.56, Math.toRadians(90));

    @Override
    public void init() {
        super.init();
        if (Passthrough.pose == null || Passthrough.alliance == null) {
            follower.setPose(DEFAULT_POSE);
            robot.setAlliance(Alliance.BLUE);
        }
        else {
            follower.setPose(Passthrough.pose);
            robot.setAlliance(Passthrough.alliance);
        }
        gamepad1.setTriggerThreshold(0.2f);
        telemetry.addData("Alliance", robot.getAlliance());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Press circle to reset robot to 9,9,90");
        telemetry.addLine("Press triangle for red, square for blue");
        if (gamepad1.bWasPressed()) //circle
            follower.setPose(DEFAULT_POSE);
        if (gamepad1.yWasPressed()) //triangle
            robot.setAlliance(Alliance.RED);
        if (gamepad1.xWasPressed()) //square
            robot.setAlliance(Alliance.BLUE);
        telemetry.addData("Alliance", robot.getAlliance());
    }

    @Override
    public void start() {
        robot.turret.turretStart();
        robot.descore.up.schedule();
        robot.stopper.close.schedule();
    }

    @Override
    public void loop() {
        super.loop();

        double speedMultiplier = slowMode ? SLOW_MODE_MULTIPLIER : 1.0;

        boolean aiming = gamepad1.right_trigger_pressed;
        double turn = aiming ? robot.getHeadingAimPower() : -gamepad1.right_stick_x * speedMultiplier * 0.8;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                turn,
                follower.pose().heading(),
                robot.getTeleOpHeadingOffset()
        );
        follower.manual(powers);

        if (gamepad1.rightBumperWasPressed())
            slowMode = !slowMode;

        if (gamepad1.leftBumperWasPressed()) {
            if (descoreDeployed)
                robot.descore.up.schedule();
            else
                robot.descore.down.schedule();
            descoreDeployed = !descoreDeployed;
        }

        boolean full = robot.transfer.isFull();
        if (full && !fullWasTrue) {
            reversing = true;
            reverseTimer.reset();
            gamepad1.rumble(500);
        }
        fullWasTrue = full;

        if (reversing && reverseTimer.milliseconds() >= REVERSE_DURATION_MS)
            reversing = false;

        if (gamepad1.b) {
            robot.transfer.on.schedule();
            robot.intake.setPower(-1.0);
        }
        else if (reversing) {
            robot.transfer.on.schedule();
            robot.intake.reverse.schedule();
        }
        else if (gamepad1.left_trigger_pressed || descoreDeployed) {
            robot.transfer.on.schedule();
            robot.intake.on.schedule();
        }
        else {
            robot.transfer.off.schedule();
            robot.intake.off.schedule();
        }

        if (gamepad1.rightTriggerWasReleased())
            robot.shoot().schedule();
    }
}