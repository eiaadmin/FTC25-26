package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Hood Servo Control")
public class HoodServoControl extends LinearOpMode {

    private static final String SERVO_NAME = "shootexpservo1";

    // Replace these with the values you found in calibration
    private static final double MIN_POS = 0;  // example: your measured bottom
    private static final double MAX_POS = 1;  // example: your measured top

    // If you measured hood angles, set these (example values):
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;

    // Presets in DEGREES (edit to taste)
    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 35;

    // Fine trim per bumper tap (degrees)
    private static final double NUDGE_DEG = 0.5;

    // Gentle motion limiter (position units per loop)
    private static final double MAX_STEP = 0.02;

    private Servo hoodServo;
    private double targetPos; // 0..1
    private double targetDeg; // for telemetry

    @Override
    public void runOpMode() {
        hoodServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // Flip if motion is reversed
        hoodServo.setDirection(Servo.Direction.REVERSE);

        targetPos = 0.5*(MIN_POS + MAX_POS);
        targetDeg = posToDeg(targetPos);
        hoodServo.setPosition(targetPos);

        telemetry.addLine("Dpad: LOW/MID/HIGH  |  Bumpers: trim  |  Stick: analog");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- Presets ---
            if (gamepad1.dpad_down) setTargetDeg(PRESET_LOW_DEG);
            if (gamepad1.dpad_left) setTargetDeg(PRESET_MID_DEG);
            if (gamepad1.dpad_up)   setTargetDeg(PRESET_HIGH_DEG);

            // --- Trim ---
            if (gamepad1.right_bumper) setTargetDeg(targetDeg + NUDGE_DEG);
            if (gamepad1.left_bumper)  setTargetDeg(targetDeg - NUDGE_DEG);

            // --- Optional analog sweep (left stick up = more angle) ---
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.10) {
                double desiredDeg = Range.scale(stick, -1.0, 1.0, HOOD_MIN_DEG, HOOD_MAX_DEG);
                setTargetDeg(desiredDeg);
            }

            // Slew-limit actual motion
            double current = hoodServo.getPosition();
            double step = Range.clip(targetPos - current, -MAX_STEP, MAX_STEP);
            double next = Range.clip(current + step, MIN_POS, MAX_POS);
            hoodServo.setPosition(next);

            // Keep degree mirror in sync
            if (Math.abs(next - targetPos) > 1e-6) {
                targetPos = next;
                targetDeg = posToDeg(targetPos);
            }

            telemetry.addData("Current Pos", "%.3f", current);
            telemetry.addData("Target  Pos", "%.3f", targetPos);
            telemetry.addData("Target  Deg", "%.1f°", targetDeg);
            telemetry.addData("Limits",      "[%.2f .. %.2f]", MIN_POS, MAX_POS);
            telemetry.update();

            sleep(10);
        }
    }

    private void setTargetDeg(double deg) {
        double clippedDeg = Range.clip(deg, HOOD_MIN_DEG, HOOD_MAX_DEG);
        targetDeg = clippedDeg;
        targetPos = Range.clip(
                Range.scale(clippedDeg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS),
                MIN_POS, MAX_POS
        );
    }

    private double posToDeg(double pos) {
        return Range.scale(pos, MIN_POS, MAX_POS, HOOD_MIN_DEG, HOOD_MAX_DEG);
    }
}
