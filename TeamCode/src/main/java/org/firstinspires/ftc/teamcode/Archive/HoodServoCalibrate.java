package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Hood Servo Calibrate")
//@Disabled
public class HoodServoCalibrate extends LinearOpMode {

    private static final String SERVO_NAME = "shootexpservo1";

    // Start conservative; you’ll widen after testing
    private static final double MIN_POS = 0.636;
    private static final double MAX_POS = 0.0380;

    private static final double NUDGE = 0.002; // tiny step per press-hold

    private Servo hoodServo;
    private double pos;

    @Override
    public void runOpMode() {
        hoodServo = hardwareMap.get(Servo.class, SERVO_NAME);

        // If motion is backwards, uncomment:
        hoodServo.setDirection(Servo.Direction.REVERSE);

        pos = 0.5 * (MIN_POS + MAX_POS);
        hoodServo.setPosition(pos);

        telemetry.addLine("Use bumpers to nudge; A = center; B = print hint.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) pos += NUDGE;          // up
            if (gamepad1.left_bumper)  pos -= NUDGE;          // down
            if (gamepad1.a)            pos = 0.5*(MIN_POS+MAX_POS);

            pos = Range.clip(pos, 0.00, 1.00);
            hoodServo.setPosition(pos);

            telemetry.addData("Commanded Pos", "%.4f", pos);
            telemetry.addLine("When at MECH bottom, record this as MIN_POS");
            telemetry.addLine("When at MECH top, record this as MAX_POS");
            if (gamepad1.b) telemetry.addLine("Tip: widen MIN/MAX gradually, never bind!");
            telemetry.update();

            sleep(10);
        }
    }
}
