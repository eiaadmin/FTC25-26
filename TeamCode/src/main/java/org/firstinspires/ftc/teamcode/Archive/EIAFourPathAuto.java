package org.firstinspires.ftc.teamcode.Archive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "FourPathAuto", group = "PedroPathing")
@Disabled
public class EIAFourPathAuto extends OpMode {

    // ----------------------------
    // Pedro follower
    // ----------------------------
    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Paths paths;

    // ----------------------------
    // Drive motors (ONLY used during RT aiming pause)
    // ----------------------------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    // ----------------------------
    // Shooter + intake hardware
    // ----------------------------
    private DcMotorEx flywheelMotor, flywheelMotor2;
    private DcMotor rollerIntakeMotor;
    private CRServo shootrollerServo;
    private Servo shootServo;
    private Servo hardstopServo;
    private Limelight3A limelight;

    // ----------------------------
    // Offsets (your current path shifting)
    // ----------------------------
    private static final double X_OFFSET = +6.0;
    private static final double Y_OFFSET = -12.0;

    private static Pose offXY(double x, double y) {
        // Keep (55, 17) exactly unchanged
        if (Math.abs(x - 55.0) < 1e-9 && Math.abs(y - 17.0) < 1e-9) {
            return new Pose(x, y);
        } else if (Math.abs(x - 19.0) < 1e-9 && Math.abs(y - 13.0) < 1e-9) {
            return new Pose(x + X_OFFSET, y); // your special-case rule
        } else {
            return new Pose(x + X_OFFSET, y + Y_OFFSET);
        }
    }

    // ----------------------------
    // Mechanism constants (TeleOp-matching hood mapping + flywheel gating)
    // ----------------------------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;
    private static final double MAX_RPM       = 4500.0;

    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0;
    private static final double FEED_REVERSE = +1.0;

    // Flywheel PIDF (same style as your TeleOp)
    private static final double FW_kP = 0.01;
    private static final double FW_kI = 0.0;
    private static final double FW_kD = 0.0;
    private double lastAppliedTPS = -1;

    // Feed gating (must be at speed before feeding)
    private static final double RESUME_RPM_FRAC = 0.85;
    private static final double PAUSE_RPM_FRAC  = 0.80;
    private boolean feedEnabled = false;

    // tx PID (dt-based)
    private static final double AIM_TOL_DEG = 1.0;
    private static final double MAX_TURN   = 0.6;

    private static final double AIM_kP = 0.035;
    private static final double AIM_kI = 0.000;
    private static final double AIM_kD = 0.002;

    private static final double I_ZONE_DEG = 5.0;
    private static final double I_MAX      = 0.2;

    private double prevError = 0.0;
    private double integral  = 0.0;
    private long   lastTsNanos = 0L;

    // ----------------------------
    // RT / LT state
    // ----------------------------
    private boolean rtPauseActive = false;
    private Timer rtTimer = new Timer();

    private boolean ltActive = false;

    // RT presets
    // After Path 1 and 4 -> 4500 RPM, 32 deg
    private static final double SHOT_A_RPM = 4500.0;
    private static final double SHOT_A_HOOD_DEG = 33.0;

    // After Path 8, 11, 14 -> 4100 RPM, 27 deg
    private static final double SHOT_B_RPM = 4100.0;
    private static final double SHOT_B_HOOD_DEG = 27.0;

    private double activeShotRPM = 0.0;
    private double activeShotHoodDeg = 0.0;

    // How long to spend in RT pause (aim + shoot)
    private static final double RT_PAUSE_SECONDS = 3;

    // =========================================================
    // Paths
    // =========================================================
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;

        public Paths(Follower follower) {

            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(55.000, 7.000), offXY(55.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(288))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            offXY(55.000, 17.000),
                            offXY(55.000, 38.000),
                            offXY(50.000, 36.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(288), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(50.000, 36.000), offXY(20.000, 36.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(20.000, 36.000), offXY(55.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(288))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(55.000, 17.000), offXY(50.000, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(288), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(50.000, 60.000), offXY(20.000, 60.000)))
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            offXY(20.000, 60.000),
                            offXY(46.000, 65.500),
                            offXY(15.000, 70.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            offXY(15.000, 70.000),
                            offXY(63.000, 71.000),
                            offXY(60.000, 84.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(60.000, 84.000), offXY(50.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(50.000, 84.000), offXY(15.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(15.000, 84.000), offXY(60.000, 84.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(315))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            offXY(60.000, 84.000),
                            offXY(67.000, 13.000),
                            offXY(19.000, 13.000)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(315), Math.toRadians(180))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(19.000, 13.000), offXY(9.000, 13.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path14 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            offXY(9.000, 13.000),
                            offXY(33.000, 9.250),
                            offXY(9.000, 7.500)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path15 = follower.pathBuilder()
                    .addPath(new BezierLine(offXY(9.000, 7.500), offXY(55.000, 17.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(288))
                    .build();
        }
    }

    // =========================================================
    // FSM helpers
    // =========================================================
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    // Intake during paths 3, 6, 10, 13, 14
    // Path3 is executed while state == 3 is being queued? Actually:
    // - We set state AFTER starting a path. So while Path3 is running, pathState == 3.
    // Same pattern for others below.
    private boolean isIntakePathState(int state) {
        return state == 3   // Path3 running
                || state == 6   // Path6 running
                || state == 10  // Path10 running
                || state == 13  // Path13 running
                || state == 14; // Path14 running
    }

    // NEW: pre-spin during Path1 and Path4 (while they are running) but DO NOT feed
    // Path1 runs while pathState == 1, Path4 runs while pathState == 4
    private boolean isPreSpinState(int state) {
        return state == 1 || state == 4;
    }

    private void beginRTPauseForFinishedPath(int finishedPathIndex) {
        rtPauseActive = true;
        rtTimer.resetTimer();

        // Choose shot preset based on which path just finished
        // finishedPathIndex is: 1,4,8,11,14 in your language,
        // but we pass the state that started those paths:
        // - after Path1 -> finishedPathIndex = 1
        // - after Path4 -> finishedPathIndex = 4
        // - after Path8 -> finishedPathIndex = 8
        // - after Path11 -> finishedPathIndex = 11
        // - after Path14 -> finishedPathIndex = 14
        if (finishedPathIndex == 1 || finishedPathIndex == 4) {
            activeShotRPM = SHOT_A_RPM;
            activeShotHoodDeg = SHOT_A_HOOD_DEG;
        } else {
            activeShotRPM = SHOT_B_RPM;
            activeShotHoodDeg = SHOT_B_HOOD_DEG;
        }

        // Hardstop open while shooting
        hardstopServo.setPosition(0.15);

        // Apply hood + keep flywheel commanded
        shootServo.setPosition(degToPos(activeShotHoodDeg));
        setFlywheelRPM(activeShotRPM);

        // Start with feeding OFF until flywheel is up to speed
        feedEnabled = false;
        rollerIntakeMotor.setPower(0);
        shootrollerServo.setPower(0);

        resetAimPid();
    }

    private void endRTPause() {
        rtPauseActive = false;

        feedEnabled = false;
        rollerIntakeMotor.setPower(0);
        shootrollerServo.setPower(0);

        hardstopServo.setPosition(0.55);

        flywheelMotor.setVelocity(0);
        flywheelMotor2.setPower(0);
        lastAppliedTPS = -1;

        activeShotRPM = 0.0;
        activeShotHoodDeg = 0.0;

        setTurnPower(0);
    }

    // =========================================================
    // Autonomous path update (RT after 1,4,8,11,14)
    // =========================================================
    private void autonomousPathUpdate() {

        if (rtPauseActive) return;
        if (follower.isBusy()) return;

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                setPathState(1);
                break;

            case 1:
                // Path1 finished -> RT pause
                beginRTPauseForFinishedPath(1);
                setPathState(101);
                break;

            case 101:
                follower.followPath(paths.Path2);
                setPathState(2);
                break;

            case 2:
                follower.followPath(paths.Path3);
                setPathState(3);
                break;

            case 3:
                follower.followPath(paths.Path4);
                setPathState(4);
                break;

            case 4:
                // Path4 finished -> RT pause
                beginRTPauseForFinishedPath(4);
                setPathState(104);
                break;

            case 104:
                follower.followPath(paths.Path5);
                setPathState(5);
                break;

            case 5:
                follower.followPath(paths.Path6);
                setPathState(6);
                break;

            case 6:
                follower.followPath(paths.Path7);
                setPathState(7);
                break;

            case 7:
                follower.followPath(paths.Path8);
                setPathState(8);
                break;

            case 8:
                // Path8 finished -> RT pause
                beginRTPauseForFinishedPath(8);
                setPathState(108);
                break;

            case 108:
                follower.followPath(paths.Path9);
                setPathState(9);
                break;

            case 9:
                follower.followPath(paths.Path10);
                setPathState(10);
                break;

            case 10:
                follower.followPath(paths.Path11);
                setPathState(11);
                break;

            case 11:
                // Path11 finished -> RT pause
                beginRTPauseForFinishedPath(11);
                setPathState(111);
                break;

            case 111:
                follower.followPath(paths.Path12);
                setPathState(12);
                break;

            case 12:
                follower.followPath(paths.Path13);
                setPathState(13);
                break;

            case 13:
                follower.followPath(paths.Path14);
                setPathState(14);
                break;

            case 14:
                // Path14 finished -> RT pause
                beginRTPauseForFinishedPath(14);
                setPathState(114);
                break;

            case 114:
                follower.followPath(paths.Path15);
                setPathState(15);
                break;

            case 15:
                setPathState(-1);
                break;

            default:
                break;
        }
    }

    // =========================================================
    // RT logic (aim with tx only, fixed RPM + hood preset)
    // + FEED GATING: only feed when flywheel is up to speed
    // =========================================================
    private void updateRTPause() {
        if (!rtPauseActive) return;

        if (rtTimer.getElapsedTimeSeconds() >= RT_PAUSE_SECONDS) {
            endRTPause();
            return;
        }

        // Keep hood + flywheel commanded
        shootServo.setPosition(degToPos(activeShotHoodDeg));
        setFlywheelRPM(activeShotRPM);

        // Aim using Limelight tx only
        LLResult ll = limelight.getLatestResult();
        boolean tagSeen = (ll != null && ll.isValid());

        double turnCmd = 0.0;
        if (tagSeen) {
            double tx = ll.getTx();
            turnCmd = pidTurnFromTx(tx);
        } else {
            resetAimPid();
            turnCmd = 0.0;
        }

        setTurnPower(turnCmd);

        // FEED GATING
        double curTPS = Math.abs(flywheelMotor.getVelocity());
        double resumeTPS = rpmToTicksPerSec(activeShotRPM * RESUME_RPM_FRAC);
        double pauseTPS  = rpmToTicksPerSec(activeShotRPM * PAUSE_RPM_FRAC);

        if (!feedEnabled && curTPS >= resumeTPS) {
            feedEnabled = true;
        } else if (feedEnabled && curTPS < pauseTPS) {
            feedEnabled = false;
        }

        if (feedEnabled) {
            rollerIntakeMotor.setPower(INTAKE_POWER);
            shootrollerServo.setPower(FEED_FORWARD);
        } else {
            rollerIntakeMotor.setPower(0);
            shootrollerServo.setPower(0);
        }

        telemetry.addData("RT tagSeen", tagSeen);
        telemetry.addData("RT turnCmd", turnCmd);
        telemetry.addData("RT shotRPM", activeShotRPM);
        telemetry.addData("RT hoodDeg", activeShotHoodDeg);
        telemetry.addData("RT curTPS", curTPS);
        telemetry.addData("RT feedEnabled", feedEnabled);
    }

    private void setTurnPower(double turnCmd) {
        turnCmd = Range.clip(turnCmd, -MAX_TURN, MAX_TURN);

        double denom = Math.max(Math.abs(turnCmd), 1.0);

        frontLeftMotor.setPower((turnCmd) / denom);
        backLeftMotor.setPower((turnCmd) / denom);
        frontRightMotor.setPower((-turnCmd) / denom);
        backRightMotor.setPower((-turnCmd) / denom);
    }

    // =========================================================
    // LT logic (intake while certain paths are running)
    // =========================================================
    private void updateLTLogic() {
        ltActive = !rtPauseActive && follower.isBusy() && isIntakePathState(pathState);

        if (ltActive) {
            rollerIntakeMotor.setPower(INTAKE_POWER);
            shootrollerServo.setPower(FEED_REVERSE);

            // shooter off while intaking
            flywheelMotor.setVelocity(0);
            flywheelMotor2.setPower(0);
            lastAppliedTPS = -1;

            hardstopServo.setPosition(0.55);
            feedEnabled = false;
        }
    }

    // =========================================================
    // NEW: Pre-spin logic (run flywheel during Path1 and Path4, but do NOT shoot)
    // =========================================================
    private void updatePreSpinLogic() {
        boolean preSpinActive = !rtPauseActive && follower.isBusy() && isPreSpinState(pathState);

        if (preSpinActive) {
            // Pre-spin uses the same preset as the stop after these paths
            activeShotRPM = SHOT_A_RPM;
            activeShotHoodDeg = SHOT_A_HOOD_DEG;

            // Keep hood staged (optional, but makes it consistent)
            shootServo.setPosition(degToPos(activeShotHoodDeg));

            // Run flywheel continuously
            setFlywheelRPM(activeShotRPM);

            // IMPORTANT: do NOT feed during the path
            feedEnabled = false;
            rollerIntakeMotor.setPower(0);
            shootrollerServo.setPower(0);

            // Keep hardstop closed while moving
            hardstopServo.setPosition(0.55);
        }
    }

    // =========================================================
    // Flywheel helpers (TeleOp style)
    // =========================================================
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }

    private void setFlywheelRPM(double rpm) {
        rpm = Math.min(rpm, MAX_RPM);
        double targetTPS = rpmToTicksPerSec(rpm);

        if (targetTPS > 1 && (lastAppliedTPS < 0 || Math.abs(targetTPS - lastAppliedTPS) > 25)) {
            double kF = 26767 / targetTPS;
            PIDFCoefficients pidf = new PIDFCoefficients(FW_kP, FW_kI, FW_kD, kF);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastAppliedTPS = targetTPS;
        }

        flywheelMotor.setVelocity(targetTPS);

        double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(MAX_RPM), 0.0, 1.0);
        flywheelMotor2.setPower(approxPower);
    }

    // Hood mapping — servo direction is set to REVERSE in init
    private double degToPos(double deg) {
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0, 1);
    }

    // =========================================================
    // Aim PID (tx)
    // =========================================================
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

    private void resetAimPid() {
        prevError = 0.0;
        integral  = 0.0;
        lastTsNanos = System.nanoTime();
    }

    // =========================================================
    // OpMode lifecycle
    // =========================================================
    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);

        // Drive motors for RT aiming pause
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Mechanisms (names copied from your TeleOp)
        shootServo       = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE); // matches TeleOp

        hardstopServo    = hardwareMap.servo.get("hardstopServo");
        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor    = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        flywheelMotor2   = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");
        rollerIntakeMotor= hardwareMap.dcMotor.get("Rollerintakeexp1");

        limelight        = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default safe positions
        hardstopServo.setPosition(0.55);
        shootServo.setPosition(0.5);

        rollerIntakeMotor.setPower(0);
        shootrollerServo.setPower(0);
        flywheelMotor.setVelocity(0);
        flywheelMotor2.setPower(0);

        // Keep your working start pose line
        follower.setStartingPose(new Pose(55.000, 7.000, Math.toRadians(270)));

        telemetry.addLine("READY");
        telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
        resetAimPid();
        rtPauseActive = false;
        feedEnabled = false;
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        // Intake while certain paths run
        updateLTLogic();

        // Pre-spin flywheel while Path1 and Path4 are running (but don't shoot)
        updatePreSpinLogic();

        // RT pause shooting/aiming (includes feed gating)
        updateRTPause();

        // If NOT in RT pause, advance path FSM
        if (!rtPauseActive) {
            if (!ltActive) {
                // default: intake off (pre-spin keeps it off too)
                rollerIntakeMotor.setPower(0);
                shootrollerServo.setPower(0);
            }
            autonomousPathUpdate();
        }

        telemetry.addData("pathState", pathState);
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("rtPauseActive", rtPauseActive);
        telemetry.addData("ltActive", ltActive);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("flywheelTPS", Math.abs(flywheelMotor.getVelocity()));
        telemetry.addData("feedEnabled", feedEnabled);
        telemetry.update();
    }

    @Override
    public void stop() {
        endRTPause();
        rollerIntakeMotor.setPower(0);
        shootrollerServo.setPower(0);
        hardstopServo.setPosition(0.55);
        setTurnPower(0);
    }
}
