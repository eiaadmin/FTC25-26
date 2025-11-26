package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "HardstopServoController", group = "Testing")
public class HardstopServoController extends LinearOpMode {
    private Servo hardstopServo;
    // === Customize These Positions ===
    private static final double POS_START = 0.15;   // starting hardstop angle
    private static final double POS_END   = 0.75;   // extended hardstop angle
    private static final double POS_MID   = 0.45;   // optional mid position
    @Override
    public void runOpMode() {
        hardstopServo = hardwareMap.get(Servo.class, "hardstopServo");
        // Initialize to start
        hardstopServo.setPosition(POS_START);
        telemetry.addLine("Hardstop Servo Controller Ready");
        telemetry.addLine("A = Move to START");
        telemetry.addLine("B = Move to END");
        telemetry.addLine("X = Move to MID");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // Move to START position
            if (gamepad1.a) {
                hardstopServo.setPosition(POS_START);
            }
            // Move to END position
            if (gamepad1.b) {
                hardstopServo.setPosition(POS_END);
            }
            // Move to MID position
            if (gamepad1.x) {
                hardstopServo.setPosition(POS_MID);
            }
            // Telemetry Updates
            telemetry.addData("Servo Position", hardstopServo.getPosition());
            telemetry.update();
        }
    }
}










