package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Tuning OpMode for the software-controlled Shooter subsystem.
 *
 * CONTROLS (gamepad 1)
 *  A ................ spin to the target
 *  B ................ stop
 *  Dpad up/down ..... target +/- 100 ticks/sec
 *  Dpad right/left .. target +/- 20 ticks/sec (hub velocity reads in 20 tick/sec steps)
 *  Right/left bumper  kS +/- 0.005
 *  Right/left stick button  kV +/- 2%
 *  Y / X ............ kP +/- 0.0005
 *  Back ............. reset the swing measurement
 *
 * Edits to kS / kV / kP only last until the OpMode ends. When you are happy with them,
 * copy the values from telemetry into the fields at the top of Shooter.java.
 */
@TeleOp
public class ShooterTuner extends OpMode {

    private static final double SWING_SETTLE_SEC = 2.0; // ignore the first seconds after a target change

    private Shooter shooter;
    private VoltageSensor battery;

    private final Gamepad cur = new Gamepad();
    private final Gamepad prev = new Gamepad();

    private final ElapsedTime sinceTargetChange = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private double target = 1200;
    private boolean spinning = false;
    private double voltage = 0;

    private double swingMin = Double.MAX_VALUE;
    private double swingMax = -Double.MAX_VALUE;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
        telemetry.addLine("A = spin, B = stop, dpad = target. Ready.");
        telemetry.update();
    }

    @Override
    public void loop() {
        prev.copy(cur);
        cur.copy(gamepad1);

        boolean targetChanged = false;

        if (pressed(cur.a, prev.a)) {
            spinning = true;
            targetChanged = true;
        }
        if (pressed(cur.b, prev.b)) {
            spinning = false;
            targetChanged = true;
        }

        if (pressed(cur.dpad_up, prev.dpad_up)) { target += 100; targetChanged = true; }
        if (pressed(cur.dpad_down, prev.dpad_down)) { target -= 100; targetChanged = true; }
        if (pressed(cur.dpad_right, prev.dpad_right)) { target += 20; targetChanged = true; }
        if (pressed(cur.dpad_left, prev.dpad_left)) { target -= 20; targetChanged = true; }
        target = Math.max(0, Math.min(3000, target));

        if (pressed(cur.right_bumper, prev.right_bumper)) Shooter.kS += 0.005;
        if (pressed(cur.left_bumper, prev.left_bumper)) Shooter.kS = Math.max(0, Shooter.kS - 0.005);
        if (pressed(cur.right_stick_button, prev.right_stick_button)) Shooter.kV *= 1.02;
        if (pressed(cur.left_stick_button, prev.left_stick_button)) Shooter.kV *= 0.98;
        if (pressed(cur.y, prev.y)) Shooter.kP += 0.0005;
        if (pressed(cur.x, prev.x)) Shooter.kP = Math.max(0, Shooter.kP - 0.0005);

        if (targetChanged || pressed(cur.back, prev.back)) {
            sinceTargetChange.reset();
            swingMin = Double.MAX_VALUE;
            swingMax = -Double.MAX_VALUE;
        }

        shooter.setTargetVelocity(spinning ? target : 0);
        shooter.update();

        double filtered = shooter.getFilteredVelocity();

        // measure how much the speed wanders once it has had time to settle
        if (spinning && sinceTargetChange.seconds() > SWING_SETTLE_SEC) {
            swingMin = Math.min(swingMin, filtered);
            swingMax = Math.max(swingMax, filtered);
        }

        // voltage reads are slow, so only refresh a few times a second
        if (voltageTimer.seconds() > 0.5) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }

        telemetry.addData("Mode", spinning ? "SPINNING" : "OFF");
        telemetry.addData("Battery (V)", "%.2f", voltage);
        telemetry.addLine();
        telemetry.addData("Target (ticks/s)", "%.0f", target);
        telemetry.addData("Actual, filtered", "%.0f", filtered);
        telemetry.addData("Actual, raw", "%.0f", shooter.getVelocity());
        telemetry.addData("Error", "%.0f", spinning ? target - filtered : 0);
        telemetry.addData("Motor power", "%.3f", shooter.getLastPower());
        telemetry.addData("At target (+/-" + (int) Shooter.closeEnough + ")", shooter.atTarget());
        telemetry.addLine();
        if (swingMax > swingMin) {
            telemetry.addData("Swing since settle", "%.0f ticks/s (min %.0f, max %.0f)",
                    swingMax - swingMin, swingMin, swingMax);
        } else {
            telemetry.addData("Swing since settle", "waiting %.1f s",
                    Math.max(0, SWING_SETTLE_SEC - sinceTargetChange.seconds()));
        }
        telemetry.addLine();
        telemetry.addData("kS", "%.4f", Shooter.kS);
        telemetry.addData("kV", "%.7f  (1/kV = %.0f)", Shooter.kV, 1.0 / Shooter.kV);
        telemetry.addData("kP", "%.4f", Shooter.kP);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.off();
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }
}