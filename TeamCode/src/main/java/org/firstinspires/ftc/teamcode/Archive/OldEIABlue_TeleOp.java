package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="EIABlue_TeleOp", group="Linear OpMode")
public class EIABlue_TeleOp extends LinearOpMode {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx flywheelMotor,flywheelMotor2;
    private DcMotor rollerIntakeMotor;
    private CRServo shootrollerServo;
    private Servo shootServo;
    private Servo hardstopServo;
    private Limelight3A limelight;
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;
    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 35.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;
    private static final double MAX_RPM       = 4500.0;
    private static final double RESUME_RPM_FRAC = 0.85;
    private static final double PAUSE_RPM_FRAC  = 0.80;
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0;
    private static final double FEED_REVERSE = +1.0;
    private static final double AIM_kP = 0.03;
    private static final double AIM_kD = 0.002;
    private double lastTx = 0;
    private boolean feedEnabled = false;
    private boolean manualOverride = false;
    private static final double DIST_A = 768.0;
    private static final double DIST_B = 10.06;
    private static final double HOOD_m = 0.30137;
    private static final double HOOD_b = 11.44;
    private static final double RPM_m = 27.40;
    private static final double RPM_b = 2267.0;
    @Override
    public void runOpMode() {
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shootServo = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);
        hardstopServo = hardwareMap.servo.get("hardstopServo");
        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor    = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        flywheelMotor2    = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");
        rollerIntakeMotor= hardwareMap.dcMotor.get("Rollerintakeexp1");
        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootServo.setPosition(0.5);
        telemetry.addLine("READY");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        limelight.start();
        double targetTPS = 0;
        while (opModeIsActive()) {
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.3;
            double rx =  gamepad1.right_stick_x;
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeftMotor.setPower((y + x + rx) / denom);
            backLeftMotor.setPower( (y - x + rx) / denom);
            frontRightMotor.setPower((y - x - rx) / denom);
            backRightMotor.setPower( (y + x - rx) / denom);
            boolean lt = gamepad1.left_trigger  > 0.1;
            boolean rt = gamepad1.right_trigger > 0.1;
            // HARDSTOP
            if (rt) hardstopServo.setPosition(0.15);
            else    hardstopServo.setPosition(0.55);
            // LIMELIGHT DATA
            LLResult ll = limelight.getLatestResult();
            boolean tagSeen = (ll != null && ll.isValid());
            double tx = 0, ty = 0;
            double autoHoodDeg = 0, autoRPM = 0;
            double turnCmd = 0;
            // =============================
            // MANUAL OVERRIDE CHECK
            // =============================
            manualOverride =
                    gamepad2.dpad_up ||
                            gamepad2.dpad_left ||
                            gamepad2.dpad_down;
            // =============================
            // AUTO SHOOTING (IF RT & TAG)
            // =============================
            if (rt && tagSeen) {
                tx = ll.getTx();
                ty = ll.getTy();
                double dist = DIST_A / (ty + DIST_B);
                if (!manualOverride) {
                    // FULL AUTO LL MODE
                    autoHoodDeg = HOOD_m * dist + HOOD_b;
                    autoHoodDeg = Range.clip(autoHoodDeg, 0, 40);
                    shootServo.setPosition(degToPos(autoHoodDeg));
                    autoRPM = RPM_m * dist + RPM_b;
                    autoRPM = Math.min(autoRPM, MAX_RPM);
                } else {
                    // =============================
                    // MANUAL OVERRIDE MODE
                    // =============================
                    autoRPM = 4500;
                    if (gamepad2.dpad_up)      autoHoodDeg = PRESET_HIGH_DEG;
                    else if (gamepad2.dpad_left)  autoHoodDeg = PRESET_MID_DEG;
                    else if (gamepad2.dpad_down)  autoHoodDeg = PRESET_LOW_DEG;
                    shootServo.setPosition(degToPos(autoHoodDeg));
                }
                targetTPS = rpmToTicksPerSec(autoRPM);
                double dTx = tx - lastTx;
                lastTx = tx;
                turnCmd = AIM_kP * tx + AIM_kD * dTx;
                frontLeftMotor.setPower((y + x + turnCmd) / denom);
                backLeftMotor.setPower( (y - x + turnCmd) / denom);
                frontRightMotor.setPower((y - x - turnCmd) / denom);
                backRightMotor.setPower( (y + x - turnCmd) / denom);
            }
            // FEED LOGIC
            if (rt) {
                flywheelMotor.setVelocity(targetTPS);
                double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
                flywheelMotor2.setPower(approxPower);
                double curTPS   = Math.abs(flywheelMotor.getVelocity());
                double resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC);
                double pauseTPS  = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC);
                if (!feedEnabled && curTPS >= resumeTPS) feedEnabled = true;
                else if (feedEnabled && curTPS < pauseTPS) feedEnabled = false;
                if (feedEnabled && tagSeen) {
                    rollerIntakeMotor.setPower(INTAKE_POWER);
                    shootrollerServo.setPower(FEED_FORWARD);
                } else {
                    rollerIntakeMotor.setPower(0);
                    shootrollerServo.setPower(0);
                }
            } else if (lt) {
                feedEnabled = false;
                flywheelMotor.setVelocity(0);
                flywheelMotor2.setVelocity(0);
                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_REVERSE);
            } else {
                feedEnabled = false;
                flywheelMotor.setVelocity(0);
                flywheelMotor2.setVelocity(0);
                rollerIntakeMotor.setPower(0);
                shootrollerServo.setPower(0);
            }
            // ***************************************
            // MANUAL PRESETS ONLY WHEN NOT AUTO AIM
            // ***************************************
            if (!rt && !manualOverride) {
                if (gamepad2.dpad_down) shootServo.setPosition(degToPos(PRESET_LOW_DEG));
                else if (gamepad2.dpad_left) shootServo.setPosition(degToPos(PRESET_MID_DEG));
                else if (gamepad2.dpad_up)   shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
            }
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("tagSeen", tagSeen);
            telemetry.addData("hood", autoHoodDeg);
            telemetry.addData("rpm", autoRPM);
            telemetry.addData("manualOverride", manualOverride);
            telemetry.update();
        }
    }
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }
    private double degToPos(double deg) {
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0, 1);
    }
}