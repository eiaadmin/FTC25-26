package org.firstinspires.ftc.teamcode.Archive;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Q_teleop", group="Linear OpMode")
public class Q_tele extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx flywheelMotor, flywheelMotor2;
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

    // ===== Flywheel PID (kF computed dynamically in RT mode ONLY) =====
    private static final double FW_kP = 8.5;
    private static final double FW_kI = 0.0;
    private static final double FW_kD = 0.0;

    // ===== tx PID (dt-based, tolerance snap, clamped) =====
    private static final double AIM_TOL_DEG = 1.0; // aligned if |tx| <= tolerance
    private static final double MAX_TURN   = 0.6;  // clamp PID output to motor power

    private static final double AIM_kP = 0.035;
    private static final double AIM_kI = 0.000;
    private static final double AIM_kD = 0.002;

    private static final double I_ZONE_DEG = 5.0;
    private static final double I_MAX      = 0.2;

    private double prevError = 0.0;
    private double integral  = 0.0;
    private long   lastTsNanos = 0L;

    private boolean feedEnabled = false;
    private boolean manualOverride = false;

    private static final double DIST_A = 768.0;
    private static final double DIST_B = 10.06;

    private static final double HOOD_m = 0.30137;
    private static final double HOOD_b = 11.44;

    private static final double RPM_m = 27.40;
    private static final double RPM_b = 2267.0;

    // PIDF update hygiene (RT mode only)
    private double lastAppliedTPS = -1;

    // ---- Animations ----

    // Tune this threshold for your mounting distance
    // If the sensor reads <= this many mm, we consider "object detected"
    private static final double PROX_THRESHOLD_MM = 75.0;

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
        flywheelMotor2   = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");

        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");

        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);

        // Motor1: velocity loop available
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor2: follower open-loop (safe even if no encoder)
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shootServo.setPosition(0.5);

        telemetry.addLine("READY");
        telemetry.update();



        waitForStart();
        resetRuntime();

        // Track last state so we only update LEDs when it changes
        boolean lastObjectDetected = false;
        boolean firstUpdate = true;

        waitForStart();
        if (isStopRequested()) return;

        limelight.start();
        resetPid();

        double targetTPS = 0;

        while (opModeIsActive()) {

            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x * 1.3;
            double rx =  gamepad1.right_stick_x;

            boolean lt = gamepad1.left_trigger  > 0.1;
            boolean rt = gamepad1.right_trigger > 0.1;

            // NEW: DPAD UP MODE (open-loop, no PIDF/velocity loop)
            boolean dpadUpOpenLoop4500 = gamepad1.dpad_up;

            // HARDSTOP
            if (rt) hardstopServo.setPosition(0.15);
            else    hardstopServo.setPosition(0.55);

            // LIMELIGHT
            LLResult ll = limelight.getLatestResult();
            boolean tagSeen = (ll != null && ll.isValid());

            double tx = 0, ty = 0;
            double autoHoodDeg = 0, autoRPM = 0;
            double turnCmd = 0;



            manualOverride =
                    gamepad2.dpad_up ||
                            gamepad2.dpad_left ||
                            gamepad2.dpad_down;

            // =========================================================
            // DPAD UP MODE:
            // - Flywheel open-loop (no PIDF, no setVelocity)
            // - Hood uses LL hood angle (if tag seen)
            // - Aiming uses tx PID (if tag seen)
            // =========================================================
            if (dpadUpOpenLoop4500) {

                autoRPM = 4500; // target display only

                if (tagSeen) {
                    tx = ll.getTx();
                    ty = ll.getTy();

                    double dist = DIST_A / (ty + DIST_B);

                    autoHoodDeg = HOOD_m * dist + HOOD_b;

                    // keep your same ty trims
                    if (ty > 0) autoHoodDeg -= 5;
                    if (ty < 0) autoHoodDeg -= 5;

                    autoHoodDeg = Range.clip(autoHoodDeg, 0, 40);
                    shootServo.setPosition(degToPos(autoHoodDeg));

                    // keep aiming
                    turnCmd = pidTurnFromTx(tx);

                    double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnCmd), 1.0);
                    frontLeftMotor.setPower((y + x + turnCmd) / denom);
                    backLeftMotor.setPower( (y - x + turnCmd) / denom);
                    frontRightMotor.setPower((y - x - turnCmd) / denom);
                    backRightMotor.setPower( (y + x - turnCmd) / denom);

                } else {
                    // no tag -> normal drive
                    resetPid();
                    double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                    frontLeftMotor.setPower((y + x + rx) / denom);
                    backLeftMotor.setPower( (y - x + rx) / denom);
                    frontRightMotor.setPower((y - x - rx) / denom);
                    backRightMotor.setPower( (y + x - rx) / denom);
                }

                // OPEN LOOP SPIN (NO PIDF / NO setVelocity)
                flywheelMotor.setPower(1.0);
                flywheelMotor2.setPower(1.0);

                // Do NOT feed in DPAD UP mode (keeps your feeder safe)
                feedEnabled = false;
                rollerIntakeMotor.setPower(0);
                shootrollerServo.setPower(0);

                // Telemetry (targetTPS not used in this mode)
                targetTPS = 0;

            } else {
                // ==========================
                // Existing AUTO SHOOTING (RT + tag)
                // ==========================
                if (rt && tagSeen) {

                    if ( ll.getTx() < 0 ) {
                        tx = ll.getTx() - 2.5;
                    }
                    if ( ll.getTx() >= 0 ){
                        tx = ll.getTx() - 1;
                    }
                    ty = ll.getTy();

                    double dist = DIST_A / (ty + DIST_B);

                    if (!manualOverride) {

                        autoHoodDeg = HOOD_m * dist + HOOD_b;
                        autoRPM     = RPM_m * dist + RPM_b;

                        if (ty >= 0) {
                            autoRPM     += 125;
                            autoHoodDeg -= 3;
                        }
                        if (ty < 0) {
                            autoRPM     -= 125;
                            autoHoodDeg -= 8;
                        }

                        autoHoodDeg = Range.clip(autoHoodDeg, 0, 40);
                        shootServo.setPosition(degToPos(autoHoodDeg));

                        autoRPM = Math.min(autoRPM, MAX_RPM);

                    } else {

                        autoRPM = 4500;

                        if (gamepad2.dpad_up)         autoHoodDeg = PRESET_HIGH_DEG;
                        else if (gamepad2.dpad_left) autoHoodDeg = PRESET_MID_DEG;
                        else if (gamepad2.dpad_down) autoHoodDeg = PRESET_LOW_DEG;

                        shootServo.setPosition(degToPos(autoHoodDeg));
                    }

                    targetTPS = rpmToTicksPerSec(autoRPM);

                    // tx PID aiming
                    turnCmd = pidTurnFromTx(tx);

                    // Inject turnCmd into drive
                    double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnCmd), 1.0);
                    frontLeftMotor.setPower((y + x + turnCmd) / denom);
                    backLeftMotor.setPower( (y - x + turnCmd) / denom);
                    frontRightMotor.setPower((y - x - turnCmd) / denom);
                    backRightMotor.setPower( (y + x - turnCmd) / denom);

                } else {
                    // Not actively aiming: reset PID so it doesn't carry state
                    resetPid();

                    // Normal driver control
                    double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                    frontLeftMotor.setPower((y + x + rx) / denom);
                    backLeftMotor.setPower( (y - x + rx) / denom);
                    frontRightMotor.setPower((y - x - rx) / denom);
                    backRightMotor.setPower( (y + x - rx) / denom);
                }

                // ==========================
                // Flywheel / feed logic (RT/ LT / idle)
                // ==========================
                if (rt) {

                    // RT MODE: apply PIDF (velocity loop) as needed so it actually spins
                    if (targetTPS > 1 && Math.abs(targetTPS - lastAppliedTPS) > 25) {
                        //double kF = 26767 / targetTPS;
                        double kF = 14.9;
                        PIDFCoefficients pidf = new PIDFCoefficients(FW_kP, FW_kI, FW_kD, kF);
                        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                        lastAppliedTPS = targetTPS;
                    }

                    flywheelMotor.setVelocity(targetTPS);

                    // Motor2 follower: open-loop approximate
                    double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
                    flywheelMotor2.setPower(approxPower);

                    double curTPS = Math.abs(flywheelMotor.getVelocity());
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
                    flywheelMotor2.setPower(0);

                    rollerIntakeMotor.setPower(INTAKE_POWER);
                    shootrollerServo.setPower(FEED_REVERSE);

                } else {

                    feedEnabled = false;
                    flywheelMotor.setVelocity(0);
                    flywheelMotor2.setPower(0);

                    rollerIntakeMotor.setPower(0);
                    shootrollerServo.setPower(0);
                }
            }



            // MANUAL PRESETS (WHEN NOT AUTO AIMING)
            if (!rt && !manualOverride) {
                if (gamepad2.dpad_down) shootServo.setPosition(degToPos(PRESET_LOW_DEG));
                else if (gamepad2.dpad_left) shootServo.setPosition(degToPos(PRESET_MID_DEG));
                else if (gamepad2.dpad_up) shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
            }

            // Actual RPM telemetry from motor1 encoder
            double curTPS_signed = flywheelMotor.getVelocity();
            double curTPS_abs = Math.abs(curTPS_signed);
            double actualRPM = (curTPS_abs / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;

            telemetry.addData("DPAD_UP open-loop", dpadUpOpenLoop4500);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("tagSeen", tagSeen);
            telemetry.addData("hood", autoHoodDeg);
            telemetry.addData("targetRPM", "%.0f", autoRPM);
            telemetry.addData("actualRPM", "%.0f", actualRPM);
            telemetry.addData("targetTPS", "%.0f", targetTPS);
            telemetry.addData("turnCmd", turnCmd);
            telemetry.addData("aligned", Math.abs(tx) <= AIM_TOL_DEG);
            telemetry.update();
        }
    }

    // ======= dt-based tx PID helper =======
    private double pidTurnFromTx(double txDeg) {
        long now = System.nanoTime();
        double dt = (now - lastTsNanos) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastTsNanos = now;

        double error = txDeg;

        if (Math.abs(error) < I_ZONE_DEG) {
            integral += error * dt;
            integral = Range.clip(integral, -I_MAX, I_MAX);
        } else {
            integral = 0.0;
        }

        double derivative = (error - prevError) / dt;
        prevError = error;

        double out = AIM_kP * error + AIM_kI * integral + AIM_kD * derivative;

        if (Math.abs(error) <= AIM_TOL_DEG) out = 0.0;

        return Range.clip(out, -MAX_TURN, MAX_TURN);
    }

    private void resetPid() {
        prevError = 0.0;
        integral  = 0.0;
        lastTsNanos = System.nanoTime();
    }

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }

    private double degToPos(double deg) {
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0, 1);
    }
}
