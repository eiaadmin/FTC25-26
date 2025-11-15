package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RobotCentric_CombinedTriggers", group="Linear OpMode")
public class RobotCentric_CombinedTriggers extends LinearOpMode {

    // -------- Drive --------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // -------- Mechanisms --------
    private DcMotorEx flywheelMotor;      // velocity control
    private DcMotor   rollerIntakeMotor;
    private CRServo   shootrollerServo;   // feeder (CR)
    private Servo     shootServo;         // hood (positional)

    // -------- Hood mapping + presets (tune these) --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;
    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 35.0;

    // -------- Flywheel velocity control --------
    private static final double TICKS_PER_REV = 28.0;  // from your motor specs
    private static final double GEAR_RATIO    = 1.0;   // motor revs per flywheel rev

    // RPM targets
    private static final double TARGET_RPM    = 4500.0; // as requested
    private static final double IDLE_RPM      = 800.0;  // as requested
    private static final boolean KEEP_IDLE_DURING_LT = false;

    // Feeding thresholds (hysteresis)
    private static final double RESUME_RPM_FRAC = 0.80; // resume feed at >= 85% of target
    private static final double PAUSE_RPM_FRAC  = 0.75; // pause feed if < 80% of target

    // Derived ticks/sec thresholds
    private static final double TARGET_TPS = rpmToTicksPerSec(TARGET_RPM);
    private static final double IDLE_TPS   = rpmToTicksPerSec(IDLE_RPM);
    private static final double RESUME_TPS = rpmToTicksPerSec(TARGET_RPM * RESUME_RPM_FRAC);
    private static final double PAUSE_TPS  = rpmToTicksPerSec(TARGET_RPM * PAUSE_RPM_FRAC);

    // -------- Intake/feeder powers --------
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0; // forward
    private static final double FEED_REVERSE = +1.0; // reverse

    // State: feeding allowed while RT is held
    private boolean feedEnabled = false;

    @Override
    public void runOpMode() {

        // Hardware map
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shootServo       = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);

        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor     = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootServo.setPosition(0.5 * (MIN_POS + MAX_POS));

        telemetry.addLine("RobotCentric_CombinedTriggers READY");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // -------- Mecanum drive --------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.3;
            double rx =  gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeftMotor.setPower((y + x + rx) / denom);
            backLeftMotor.setPower( (y - x + rx) / denom);
            frontRightMotor.setPower((y - x - rx) / denom);
            backRightMotor.setPower( (y + x - rx) / denom);

            // -------- Trigger logic with dip-detection & hysteresis --------
            boolean lt = gamepad1.left_trigger  > 0.10;
            boolean rt = gamepad1.right_trigger > 0.10;

            if (rt) {
                // Command target velocity
                flywheelMotor.setVelocity(TARGET_TPS);
                telemetry.addData("TARGET_TPS ", TARGET_TPS);
                // Measure current speed
                double tps = Math.abs(flywheelMotor.getVelocity());
                telemetry.addData("tps ", tps);

                // Hysteresis:
                // - If currently NOT feeding, enable once we cross RESUME_TPS.
                // - If currently feeding, pause if we dip below PAUSE_TPS.
                if (!feedEnabled && tps >= RESUME_TPS) {
                    feedEnabled = true;     // first time at speed -> start/continue feeding
                } else if (feedEnabled && tps < PAUSE_TPS) {
                    feedEnabled = false;    // dip detected -> pause feeding until back up to RESUME_TPS
                }

                if (feedEnabled) {
                    rollerIntakeMotor.setPower(INTAKE_POWER);
                    shootrollerServo.setPower(FEED_FORWARD);
                } else {
                    rollerIntakeMotor.setPower(0.0);
                    shootrollerServo.setPower(0.0);
                }

            } else if (lt) {
                // Intake/unjam mode — flywheel OFF (per your setting), intake on, feeder reverse
                feedEnabled = false;
                flywheelMotor.setVelocity(0.0);

                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_REVERSE);

            } else {
                // Idle
                feedEnabled = false;

                flywheelMotor.setVelocity(IDLE_TPS);
                rollerIntakeMotor.setPower(0.0);
                shootrollerServo.setPower(0.0);
            }

            if (gamepad1.right_bumper){
                //flywheelMotor.setVelocity(TARGET_TPS);
                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_FORWARD);
            }
            // -------- Hood presets --------
            if (gamepad2.dpad_down) {
                shootServo.setPosition(degToPos(PRESET_LOW_DEG));
            } else if (gamepad2.dpad_left) {
                shootServo.setPosition(degToPos(PRESET_MID_DEG));
            } else if (gamepad2.dpad_up) {
                shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
            }

            // -------- Telemetry --------
            double curTPS = Math.abs(flywheelMotor.getVelocity());
            double curRPM = ticksPerSecToRpm(curTPS);
            String mode;
            if (rt) mode = feedEnabled ? "SHOOT: FEEDING" : "SHOOT: SPINNING (PAUSED FEED)";
            else if (lt) mode = "INTAKE (FW OFF)";
            else mode = "IDLE RPM";

            telemetry.addData("Mode", mode);
            telemetry.addData("LT / RT", "%.2f / %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("Flywheel RPM", "%.0f (target %.0f / idle %.0f)", curRPM, TARGET_RPM, IDLE_RPM);
            telemetry.addData("Resume/Pause RPM", "%.0f / %.0f", TARGET_RPM*RESUME_RPM_FRAC, TARGET_RPM*PAUSE_RPM_FRAC);
            telemetry.addData("Feed Enabled", feedEnabled);
            telemetry.addData("Intake Power", "%.2f", rollerIntakeMotor.getPower());
            telemetry.addData("Feeder Pwr",  "%.2f", shootrollerServo.getPower());
            telemetry.addData("Hood Pos",    "%.3f", shootServo.getPosition());
            telemetry.update();
        }
    }

    // -------- Helpers --------
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }
    private static double ticksPerSecToRpm(double tps) {
        double mechTpr = TICKS_PER_REV * GEAR_RATIO;
        return (tps / mechTpr) * 60.0;
    }
    private double degToPos(double deg){
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0.0, 1.0);
    }
}