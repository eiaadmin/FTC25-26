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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name="HoodCompTest", group="Linear OpMode")
public class HoodCompTest extends LinearOpMode {

    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx flywheelMotor, flywheelMotor2;
    private DcMotor rollerIntakeMotor, rollerIntakeMotor2;
    private CRServo shootrollerServo;
    private Servo shootServo;
    private Servo hardstopServo;
    private Limelight3A limelight;
    double curTPS_signed_local,curTPS_abs_local,actualRPM_local;

    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 50.0;

    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 35.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;
    private static final double MAX_RPM       = 4500.0;

    private static final double IDLE_RPM       = 4150;//2700;

    private static final double RESUME_RPM_FRAC = 0.85;
    private static final double PAUSE_RPM_FRAC  = 0.80;

    private static final double RESUME_RPM_FRAC_FAR = 0.85;
    private static final double PAUSE_RPM_FRAC_FAR  = 0.80;

    private static final double INTAKE_POWER = 1.0;//.75;
    private static final double FEED_FORWARD = -1.0;
    private static final double FEED_REVERSE = +1.0;

    // ===== Flywheel PID (kF computed dynamically in RT mode ONLY) =====
    private static final double FW_kP = 440;
    private static final double FW_kI = 0.0;
    private static final double FW_kD = 0.0;

    // ===== tx PID (dt-based, tolerance snap, clamped) =====
    private static final double AIM_TOL_DEG = 1.5; // aligned if |tx| <= tolerance
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

    private static final double PROX_THRESHOLD_MM = 65.0;

    // =========================================================
    // RPM DIP COMPENSATION (hood trim based on RPM droop)
    // =========================================================
    private static final double RPM_FILTER_ALPHA = 0.20;       // 0.10..0.30
    private static final double RPM_DEADBAND     = 75.0;       // ignore small error/noise
    private static final double HOOD_COMP_DEG_PER_RPM = 0.00070; // (kept, unused by new logic)
    private static final double HOOD_COMP_MAX_DEG     = 6.0;    // limit trim

    // NEW: rounding step (nearest 0.5 deg)
    private static final double HOOD_STEP_DEG = 0.5;

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
        rollerIntakeMotor2 = hardwareMap.dcMotor.get("Rollerintakeexp2");

        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);//0

        // Motor1: velocity loop available
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor2: follower open-loop (safe even if no encoder)
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
            else    hardstopServo.setPosition(0.40);//0.55);

            LLResult ll = limelight.getLatestResult();
            boolean tagSeen = (ll != null && ll.isValid());

            double tx = 0, ty = 0;
            double autoHoodDeg = 0, autoRPM = 0;
            double turnCmd = 0;

            // Debug telemetry for new hood comp
            double hoodCompDegTelemetry = 0.0;
            double rpmDropTelemetry = 0.0;
            double hoodBaseDegTelemetry = 0.0;
            double hoodFinalDegTelemetry = 0.0;

            manualOverride =
                    gamepad2.dpad_up ||
                            gamepad2.dpad_left ||
                            gamepad2.dpad_down;

            // ==========================
            // AUTO SHOOTING (RT + tag)
            // SolversLib InterpLUT (NO TY CLAMPING)
            // + NEW RPM dip -> hood compensation (YOUR PSEUDOCODE + rounding)
            // ==========================
            if (rt && tagSeen) {

                tx = ll.getTx();

                //if from far on blue it goes to the left of the goal (towards the trashcans) increase -3 try -2.5.
                //if from far on blue it goes to the right of the goal (towards the table) decrease -3 try -3.5.
                //if from close on blue it goes to the left of the goal (towards the trashcans) increase -1.2 try -1
                //if from close on blue it goes to the right of the goal (towards the table) decrease -1.2 try -1.5
                if ( ll.getTy() <= -16.85 ) {
                    tx = ll.getTx() - 5;
                }
                if ( ll.getTy() > -16.85 ){
                    tx = ll.getTx() - 1.2;
                }

                ty = ll.getTy();

                if (!manualOverride) {

                    autoHoodDeg = tyToHoodDeg.get(ty);
                    autoRPM     = tyToRPM.get(ty);

                    hoodBaseDegTelemetry = autoHoodDeg;

                    // ---- RPM droop compensation (NEW) ----
                    // actual RPM from motor1 encoder
                    curTPS_signed_local = flywheelMotor.getVelocity();
                    curTPS_abs_local = Math.abs(curTPS_signed_local);
                    double rpm1 = (Math.abs(flywheelMotor.getVelocity()) / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
                    double rpm2 = (Math.abs(flywheelMotor2.getVelocity()) / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
                    double actualRPM_local = (rpm1 + rpm2) / 2.0;

                    telemetry.addData("Flywheel Motor Velocity: ",curTPS_signed_local);
                    telemetry.addData("curTPS_abs_local: ",curTPS_abs_local);
                    telemetry.addData("actualRPM_local: ",actualRPM_local);

                    // low-pass filter RPM to avoid jitter
                    if (filteredRPM <= 1.0) filteredRPM = actualRPM_local; // init
                    filteredRPM = (1.0 - RPM_FILTER_ALPHA) * filteredRPM + RPM_FILTER_ALPHA * actualRPM_local;

                    // positive when wheel is below target
                    double rpmDrop = autoRPM - filteredRPM;
                    rpmDropTelemetry = rpmDrop;

                    // deadband
                    if (rpmDrop < RPM_DEADBAND) rpmDrop = 0.0;

                    // YOUR LOGIC:
                    // if actualRPM <= targetRPM:
                    //    hoodDeg -= (targetRPM - actualRPM) / 100
                    if (rpmDrop > 0.0) {
                        double hoodDeltaDeg = rpmDrop / 140.0; // <- /100 like your pseudocode
                        hoodDeltaDeg = Range.clip(hoodDeltaDeg, 0.0, HOOD_COMP_MAX_DEG);

                        autoHoodDeg += hoodDeltaDeg;           // SUBTRACT as requested
                        hoodCompDegTelemetry = hoodDeltaDeg;
                    }

                    // Round to nearest 0.5 degree
                    autoHoodDeg = roundToStep(autoHoodDeg, HOOD_STEP_DEG);

                    // Clamp and apply
                    autoHoodDeg = Range.clip(autoHoodDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);
                    hoodFinalDegTelemetry = autoHoodDeg;

                    shootServo.setPosition(degToPos(autoHoodDeg));

                    autoRPM = Math.min(autoRPM, MAX_RPM);

                } else {

                    autoRPM = 4500;

                    if (gamepad2.dpad_up)         autoHoodDeg = PRESET_HIGH_DEG;
                    else if (gamepad2.dpad_left) autoHoodDeg = PRESET_MID_DEG;
                    else if (gamepad2.dpad_down) autoHoodDeg = PRESET_LOW_DEG;

                    autoHoodDeg = roundToStep(autoHoodDeg, HOOD_STEP_DEG);
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
                    double kF = 19;
                    PIDFCoefficients pidf = new PIDFCoefficients(FW_kP, FW_kI, FW_kD, kF);
                    flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                    flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                    lastAppliedTPS = targetTPS;
                }

                flywheelMotor.setVelocity(targetTPS);
                flywheelMotor2.setVelocity(targetTPS);

                double curTPS = Math.abs(flywheelMotor.getVelocity());
                double resumeTPS;
                double pauseTPS;

                if ( ty < -14 ) {
                    resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC_FAR);
                    pauseTPS = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC_FAR);
                } else {
                    resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC);
                    pauseTPS = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC);
                }

                if (!feedEnabled && curTPS >= resumeTPS) feedEnabled = true;
                else if (feedEnabled && curTPS < pauseTPS) feedEnabled = false;

                if (feedEnabled && tagSeen && Math.abs(tx) <= AIM_TOL_DEG) {
                    rollerIntakeMotor.setPower(INTAKE_POWER);
                    rollerIntakeMotor2.setPower(INTAKE_POWER);
                    shootrollerServo.setPower(FEED_FORWARD);
                } else {
                    rollerIntakeMotor.setPower(0);
                    rollerIntakeMotor2.setPower(0);
                    shootrollerServo.setPower(0);
                }

            } else if (lt) {

                feedEnabled = false;
                targetTPS = rpmToTicksPerSec(IDLE_RPM);
                flywheelMotor.setVelocity(targetTPS);
                flywheelMotor2.setVelocity(targetTPS);

                rollerIntakeMotor.setPower(INTAKE_POWER);
                rollerIntakeMotor2.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_REVERSE);

            } else {

                feedEnabled = false;
                targetTPS = rpmToTicksPerSec(IDLE_RPM);
                flywheelMotor.setVelocity(targetTPS);
                flywheelMotor2.setVelocity(targetTPS);

                rollerIntakeMotor.setPower(0);
                rollerIntakeMotor2.setPower(0);
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

            telemetry.addData("hoodBaseDeg", "%.2f", hoodBaseDegTelemetry);
            telemetry.addData("hoodCompDeg", "%.2f", hoodCompDegTelemetry);
            telemetry.addData("hoodFinalDeg", "%.2f", hoodFinalDegTelemetry);

            telemetry.addData("targetRPM", "%.0f", autoRPM);
            double rpm1 = (Math.abs(flywheelMotor.getVelocity()) / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
            double rpm2 = (Math.abs(flywheelMotor2.getVelocity()) / (TICKS_PER_REV * GEAR_RATIO)) * 60.0;
            telemetry.addData("RPM1", "%.0f", rpm1);
            telemetry.addData("RPM2", "%.0f", rpm2);
            telemetry.addData("Mismatch", "%.0f", (rpm1 - rpm2));
            telemetry.addData("filtRPM", "%.0f", filteredRPM);
            telemetry.addData("rpmDrop", "%.0f", rpmDropTelemetry);

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
        tyToRPM.add(-100, 3950);
        tyToRPM.add(-20, 3950);
        tyToRPM.add(-16.85, 3950);
        tyToRPM.add(-16.84, 3400);
        tyToRPM.add(-15.49, 3400);
        tyToRPM.add(-15.22, 3400);
        tyToRPM.add(-15.00, 3400);
        tyToRPM.add(-14.9, 3250);
        tyToRPM.add(-14.6, 3250);
        tyToRPM.add(-14.23, 3250);
        tyToRPM.add( -13.13, 3225);
        tyToRPM.add( -11.44, 3050);
        tyToRPM.add( -8, 3050);
        tyToRPM.add(-4.75, 2800);
        tyToRPM.add(0, 2700);
        tyToRPM.add(12, 2600);
        tyToRPM.add(100.00, 2800);

        // Ty -> Hood(deg) (ascending Ty)
        tyToHoodDeg.add(-100, 41);
        tyToHoodDeg.add(-20, 41);//-2.8
        tyToHoodDeg.add(-16.85, 41);//-2.8
        tyToHoodDeg.add(-16.84, 41);//-2.8
        tyToHoodDeg.add(-15.49, 41);
        tyToHoodDeg.add(-15.22, 41);
        tyToHoodDeg.add(-15.00, 41);
        tyToHoodDeg.add(-14.9, 40);
        tyToHoodDeg.add(-14.6, 40);
        tyToHoodDeg.add(-14.23, 36);
        tyToHoodDeg.add( -13.13, 36);
        tyToHoodDeg.add( -11.44, 34);//32
        tyToHoodDeg.add( -8, 32);//29
        tyToHoodDeg.add(-4.75, 22);
        tyToHoodDeg.add(0, 16);
        tyToHoodDeg.add(12, 8);//16.00
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

    private static double roundToStep(double value, double stepDeg) {
        if (stepDeg <= 0) return value;
        return Math.round(value / stepDeg) * stepDeg;
    }
}
