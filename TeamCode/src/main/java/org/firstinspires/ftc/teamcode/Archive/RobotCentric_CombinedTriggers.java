package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="RobotCentric_CombinedTriggers", group="Linear OpMode")
@Disabled
public class RobotCentric_CombinedTriggers extends LinearOpMode {
    // -------- Drive Motors --------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    // -------- Mechanisms --------
    private DcMotorEx flywheelMotor;
    private DcMotor   rollerIntakeMotor;
    private CRServo   shootrollerServo;
    private Servo     shootServo;
    // -------- Hardstop Servo (STATEFUL) --------
    private Servo hardstopServo;
    private double hardstopPos = 0.75;   // DEFAULT POSITION (not forced)
    // -------- Hood Presets --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;
    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 60;//35.0;
    // -------- Flywheel RPM Control --------
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;
    private static final double TARGET_RPM    = 4500.0;
    private static final double IDLE_RPM      = 1000;
    private static final double RESUME_RPM_FRAC = 0.80;
    private static final double PAUSE_RPM_FRAC  = 0.75;
    private static final double TARGET_TPS = rpmToTicksPerSec(TARGET_RPM);
    private static final double IDLE_TPS   = rpmToTicksPerSec(IDLE_RPM);
    private static final double RESUME_TPS = rpmToTicksPerSec(TARGET_RPM * RESUME_RPM_FRAC);
    private static final double PAUSE_TPS  = rpmToTicksPerSec(TARGET_RPM * PAUSE_RPM_FRAC);
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0;
    private static final double FEED_REVERSE = +1.0;
    private boolean feedEnabled = false;
    // -------- Limelight --------
    private Limelight3A limelight;
    // -------- TX PID --------
    private double kP = 0.03;
    private double kD = 0.002;
    private double prevError = 0;
    private long lastTime = System.nanoTime();
    private static final double DEADZONE = 0.2;
    private static final double MAX_TURN = 0.6;
    @Override
    public void runOpMode() {
        // ===== DRIVE HARDWARE =====
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // ===== MECHANISM HARDWARE =====
        shootServo       = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);
        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor     = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootServo.setPosition(0.5);
        // ===== HARDSTOP SERVO =====
        hardstopServo = hardwareMap.get(Servo.class, "hardstopServo");
        hardstopServo.setPosition(hardstopPos);
        // ===== LIMELIGHT =====
        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addLine("TeleOp READY");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            // ---------------------
            // Driver Input
            // ---------------------
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.3;
            double rx =  gamepad1.right_stick_x;
            boolean lt = gamepad1.left_trigger  > 0.10;
            boolean rt = gamepad1.right_trigger > 0.10;
            // ==================================================
            // HARDSTOP SERVO — STATEFUL BEHAVIOR
            // ==================================================
            if (rt) {
                hardstopPos = 0.15;   // retract for shooting
            } else if (lt) {
                hardstopPos = 0.55;// extend for intake/unjam
            }
            // Apply stored position EVERY LOOP
            hardstopServo.setPosition(hardstopPos);
            // ==================================================
            // LIMELIGHT AUTO-ROTATION (RT only)
            // ==================================================
            if (rt) {
                LLResult res = limelight.getLatestResult();
                if (res != null && res.isValid() && Math.abs(res.getTx()) > 0.05) {
                    rx = pidTx(res.getTx());
                }
            }
            // ==================================================
            // MECANUM DRIVE (your known-good version)
            // ==================================================
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeftMotor.setPower((y + x + rx) / denom);
            backLeftMotor.setPower( (y - x + rx) / denom);
            frontRightMotor.setPower((y - x - rx) / denom);
            backRightMotor.setPower( (y + x - rx) / denom);
            // ==================================================
            // SHOOTER CONTROL
            // ==================================================
            if (rt) {  // Shooting mode
                flywheelMotor.setVelocity(TARGET_TPS);
                double tps = Math.abs(flywheelMotor.getVelocity());
                if (!feedEnabled && tps >= RESUME_TPS) feedEnabled = true;
                else if (feedEnabled && tps < PAUSE_TPS) feedEnabled = false;
                if (feedEnabled) {
                    rollerIntakeMotor.setPower(INTAKE_POWER);
                    shootrollerServo.setPower(FEED_FORWARD);
                } else {
                    rollerIntakeMotor.setPower(0.0);
                    shootrollerServo.setPower(0.0);
                }
            } else if (lt) {  // Intake/unjam
                feedEnabled = false;
                flywheelMotor.setVelocity(0.0);
                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_REVERSE);
            } else {  // Idle
                feedEnabled = false;
                flywheelMotor.setVelocity(IDLE_TPS);
                rollerIntakeMotor.setPower(0.0);
                shootrollerServo.setPower(0.0);
            }
            // HOOD PRESETS
            if (gamepad2.dpad_down)  shootServo.setPosition(degToPos(PRESET_LOW_DEG));
            if (gamepad2.dpad_left)  shootServo.setPosition(degToPos(PRESET_MID_DEG));
            if (gamepad2.dpad_up)    {
                shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
                telemetry.addData("shooter hood pos ",degToPos(PRESET_HIGH_DEG));
                telemetry.update();
            }
            // TELEMETRY
            LLResult dbg = limelight.getLatestResult();
            telemetry.addData("Hardstop", hardstopPos);
            telemetry.addData("LL tx", dbg != null ? dbg.getTx() : 0);
            telemetry.addData("LL ta", dbg != null ? dbg.getTa() : 0);
            telemetry.update();
        }
    }
    // ===== Helper Methods =====
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }
    private static double ticksPerSecToRpm(double tps) {
        return (tps / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
    }
    private double degToPos(double deg){
        return Range.clip(
                Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS),
                0.0, 1.0
        );
    }
    // ===== Limelight PID =====
    private double pidTx(double errorDeg) {
        if (Math.abs(errorDeg) < DEADZONE) return 0;
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (errorDeg - prevError) / dt;
        prevError = errorDeg;
        double output = kP * errorDeg + kD * derivative;
        return Range.clip(output, -MAX_TURN, MAX_TURN);
    }
}