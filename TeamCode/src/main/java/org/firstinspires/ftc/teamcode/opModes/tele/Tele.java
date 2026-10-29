package org.firstinspires.ftc.teamcode.opModes.tele;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.base.RobotOpMode;
import org.firstinspires.ftc.teamcode.control.Alliance;
import org.firstinspires.ftc.teamcode.control.Passthrough;

@TeleOp
public class Tele extends RobotOpMode {
    private boolean slowMode;
    private final double slowModeMultiplier = 0.25;

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
        telemetry.update();
    }

    @Override
    public void loop() {
        super.loop();
        double speedMultiplier = slowMode ? slowModeMultiplier : 1.0;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y * speedMultiplier,
                -gamepad1.left_stick_x * speedMultiplier,
                -gamepad1.right_stick_x * speedMultiplier,
                follower.pose().heading(),
                robot.getTeleOpHeadingOffset()
        );
        follower.manual(powers);

        if (gamepad1.dpadLeftWasPressed()) {
            slowMode = !slowMode;
        }

        if (gamepad1.left_trigger > 0.2) {
            robot.transfer.on();
            robot.intake.on();
        }
        else {
            robot.transfer.off();
            robot.intake.off();
        }

        if (gamepad1.right_trigger > 0.2)
            robot.stopper.open();
        else
            robot.stopper.close();
    }
}
