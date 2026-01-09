package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.util.InterpLUT;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;



@TeleOp(name="Q_tele", group="Linear OpMode")
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

    private static final double IDLE_RPM       = 2500;

    private static final double RESUME_RPM_FRAC = 0.75;
    private static final double PAUSE_RPM_FRAC  = 0.70;

    private static final double RESUME_RPM_FRAC_FAR = 0.90;
    private static final double PAUSE_RPM_FRAC_FAR  = 0.85;


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

    // ===== SolversLib InterpLUTs: Ty -> RPM and Ty -> HoodAngle(deg) =====
    private final InterpLUT tyToRPM = new InterpLUT();
    private final InterpLUT tyToHoodDeg = new InterpLUT();

    // PIDF update hygiene (RT mode only)
    private double lastAppliedTPS = -1;

    // ---- Sensor ----
    private GoBildaPrismDriver prism;

    // ---- Sensor ----
    private RevColorSensorV3 colorSense;

    // ---- Animations ----
    private final PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Solid solidBlue  = new PrismAnimations.Solid(Color.BLUE);


    private static final double PROX_THRESHOLD_MM = 75.0;

    // =========================================================
    // RPM DIP COMPENSATION (hood trim based on RPM droop)
    // =========================================================
    private static final double RPM_FILTER_ALPHA = 0.20;       // 0.10..0.30
    private static final double RPM_DEADBAND     = 75.0;       // ignore small error/noise
    private static final double HOOD_COMP_DEG_PER_RPM = 0.00070; // start ~0.0010
    private static final double HOOD_COMP_MAX_DEG     = 2.0;    // limit trim

    private double filteredRPM = 0.0;

    @Override
    public void runOpMode() {

        // ===== Build LUTs (Ty -> RPM / Hood) =====
        initTyLUTs();

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
        limelight.pipelineSwitch(4);

        // Motor1: velocity loop available
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor2: follower open-loop (safe even if no encoder)
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorSense = hardwareMap.get(RevColorSensorV3.class, "colorsense");

        // Prism strip length (0..119 = 120 LEDs)
        prism.setStripLength(120);

        // Configure GREEN (full strip)
        solidGreen.setBrightness(100);
        solidGreen.setStartIndex(0);
        solidGreen.setStopIndex(119);

        // Configure BLUE (full strip)
        solidBlue.setBrightness(100);
        solidBlue.setStartIndex(0);
        solidBlue.setStopIndex(119);

        //shootServo.setPosition(0.5);

        telemetry.addLine("READY");
        telemetry.update();

        waitForStart();
        resetRuntime();

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

            boolean dpadUpOpenLoop4500 = gamepad1.dpad_up;

            // HARDSTOP
            if (rt) hardstopServo.setPosition(0.15);
            else    hardstopServo.setPosition(0.55);

            LLResult ll = limelight.getLatestResult();
            boolean tagSeen = (ll != null && ll.isValid());

            double tx = 0, ty = 0;
            double autoHoodDeg = 0, autoRPM = 0;
            double turnCmd = 0;

            manualOverride =
                    gamepad2.dpad_up ||
                            gamepad2.dpad_left ||
                            gamepad2.dpad_down;

            // ==========================
            // AUTO SHOOTING (RT + tag)
            // SolversLib InterpLUT (NO TY CLAMPING)
            // + RPM dip -> hood compensation
            // ==========================
            if (rt && tagSeen) {

                if ( ll.getTy() < -1 ) {
                    tx = ll.getTx() - 1.8;
                }
                if ( ll.getTy() >= 0 ){
                    tx = ll.getTx() - 1.5;
                }
                ty = ll.getTy();

                if (!manualOverride) {

                    autoHoodDeg = tyToHoodDeg.get(ty);
                    autoRPM     = tyToRPM.get(ty);

                    // ---- RPM droop compensation (adds hood angle when RPM dips) ----
                    // actual RPM from motor1 encoder
                    double curTPS_signed_local = flywheelMotor.getVelocity();
                    double curTPS_abs_local = Math.abs(curTPS_signed_local);
                    double actualRPM_local = (curTPS_abs_local / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;

                    // low-pass filter RPM to avoid jitter
                    if (filteredRPM <= 1.0) filteredRPM = actualRPM_local; // init
                    filteredRPM = (1.0 - RPM_FILTER_ALPHA) * filteredRPM + RPM_FILTER_ALPHA * actualRPM_local;

                    // positive when wheel is below target
                    double rpmError = autoRPM - filteredRPM;

                    // deadband
                    if (rpmError < RPM_DEADBAND) rpmError = 0.0;

                    // convert to hood degrees (+)
                    double hoodComp = rpmError * HOOD_COMP_DEG_PER_RPM;
                    hoodComp = Range.clip(hoodComp, 0.0, HOOD_COMP_MAX_DEG);

                    autoHoodDeg += hoodComp;
                    // --------------------------------------------------------------

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

                turnCmd = pidTurnFromTx(tx);

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turnCmd), 1.0);
                frontLeftMotor.setPower((y + x + turnCmd) / denom);
                backLeftMotor.setPower( (y - x + turnCmd) / denom);
                frontRightMotor.setPower((y - x - turnCmd) / denom);
                backRightMotor.setPower( (y + x - turnCmd) / denom);

            } else {
                resetPid();

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                frontLeftMotor.setPower((y + x + rx) / denom);
                backLeftMotor.setPower( (y - x + rx) / denom);
                frontRightMotor.setPower( (y - x - rx) / denom);
                backRightMotor.setPower(  (y + x - rx) / denom);
            }

            // ==========================
            // Flywheel / feed logic (RT/ LT / idle)
            // ==========================
            if (rt) {

                if (targetTPS > 1 && Math.abs(targetTPS - lastAppliedTPS) > 25) {
                    double kF = 14.9;
                    PIDFCoefficients pidf = new PIDFCoefficients(FW_kP, FW_kI, FW_kD, kF);
                    flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                    lastAppliedTPS = targetTPS;
                }

                flywheelMotor.setVelocity(targetTPS);

                double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
                flywheelMotor2.setPower(approxPower);

                double curTPS = Math.abs(flywheelMotor.getVelocity());
                double resumeTPS;
                double pauseTPS;

                if ( ty < -1 ) {
                    resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC_FAR);
                    pauseTPS = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC_FAR);
                }else{
                    resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC);
                    pauseTPS = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC);
                }


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
                targetTPS = rpmToTicksPerSec(IDLE_RPM);
                flywheelMotor.setVelocity(targetTPS);

                double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
                flywheelMotor2.setPower(approxPower);

                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_REVERSE);

            } else {

                feedEnabled = false;
                targetTPS = rpmToTicksPerSec(IDLE_RPM);
                flywheelMotor.setVelocity(targetTPS);

                double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
                flywheelMotor2.setPower(approxPower);

                rollerIntakeMotor.setPower(0);
                shootrollerServo.setPower(0);
            }

            if (!rt && !manualOverride) {
                if (gamepad2.dpad_down) shootServo.setPosition(degToPos(PRESET_LOW_DEG));
                else if (gamepad2.dpad_left) shootServo.setPosition(degToPos(PRESET_MID_DEG));
                else if (gamepad2.dpad_up) shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
            }

            double curTPS_signed = flywheelMotor.getVelocity();
            double curTPS_abs = Math.abs(curTPS_signed);
            double actualRPM = (curTPS_abs / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;

            double distMm = colorSense.getDistance(DistanceUnit.MM);
            boolean objectDetected = distMm <= PROX_THRESHOLD_MM;

            // Update Prism only on change (or first loop)
            if (firstUpdate || objectDetected != lastObjectDetected) {
                prism.clearAllAnimations();

                if (objectDetected) {
                    // Object close -> GREEN
                    prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidGreen);
                } else {
                    // Otherwise -> BLUE
                    prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlue);
                }

                lastObjectDetected = objectDetected;
                firstUpdate = false;
            }

            telemetry.addData("DPAD_UP open-loop", dpadUpOpenLoop4500);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("tagSeen", tagSeen);
            telemetry.addData("hood", autoHoodDeg);
            telemetry.addData("targetRPM", "%.0f", autoRPM);
            telemetry.addData("actualRPM", "%.0f", actualRPM);
            telemetry.addData("filtRPM", "%.0f", filteredRPM);
            telemetry.addData("rpmErr", "%.0f", (autoRPM - filteredRPM));
            telemetry.addData("targetTPS", "%.0f", targetTPS);
            telemetry.addData("turnCmd", turnCmd);
            telemetry.addData("aligned", Math.abs(tx) <= AIM_TOL_DEG);
            telemetry.update();
        }
    }

    // ======= Build the SolversLib InterpLUTs =======
    private void initTyLUTs() {
        // MUST be strictly increasing X (Ty) values!

        // Ty -> RPM (ascending Ty)
        tyToRPM.add(-100, 4300);
        tyToRPM.add(-2.8, 4200);
        tyToRPM.add(-2.65, 4200);
        tyToRPM.add(-2.60, 4200);
        tyToRPM.add(-2.15, 4200);
        tyToRPM.add(-2.03, 4200);
        tyToRPM.add(-1.67, 4200);
        tyToRPM.add( 0.33, 4200);
        tyToRPM.add( 3.26, 3900);
        tyToRPM.add( 9.40, 3700);
        tyToRPM.add(16.00, 3500);
        tyToRPM.add(100.00, 3500);

        // Ty -> Hood(deg) (ascending Ty)
        tyToHoodDeg.add(-100, 40);
        tyToHoodDeg.add(-2.8, 40);
        tyToHoodDeg.add(-2.65, 40);
        tyToHoodDeg.add(-2.60, 40);
        tyToHoodDeg.add(-2.15, 40);
        tyToHoodDeg.add(-2.03, 40);
        tyToHoodDeg.add(-1.67, 35);
        tyToHoodDeg.add( 0.33, 35);
        tyToHoodDeg.add( 3.26, 32);
        tyToHoodDeg.add( 9.40, 29);
        tyToHoodDeg.add(16.00, 22);
        tyToHoodDeg.add(100.00, 0);

        tyToRPM.createLUT();
        tyToHoodDeg.createLUT();
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
