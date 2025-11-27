package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@Autonomous(name="PP_FullAuto_WithScoring", group="Auto")
public class PP_FullAuto_WithScoring extends OpMode {
    // Follower
    private Follower follower;
    // Robot hardware (same as TeleOp)
    private DcMotorEx flywheelMotor;
    private DcMotor rollerIntakeMotor;
    private CRServo shootrollerServo;
    private Servo shootServo;
    private Servo hardstopServo;
    private Limelight3A limelight;
    // Timers
    private Timer stateTimer = new Timer();
    private Timer scoreTimer = new Timer();
    // FSM
    private int state = 0;
    private boolean scoringPause = false;
    // Path storage
    private PathChain[] path = new PathChain[10];
    // Shooter constants from TeleOp
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0;
    private static final double FEED_REVERSE = +1.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double GEAR_RATIO    = 1.0;
    private static final double MAX_RPM       = 4500.0;
    private static final double RESUME_RPM_FRAC = 0.85;
    private static final double PAUSE_RPM_FRAC  = 0.80;
    private static final double DIST_A = 768.0;
    private static final double DIST_B = 10.06;
    private static final double HOOD_m = 0.3846;
    private static final double HOOD_b = 6.67;
    private static final double RPM_m = 32.69;
    private static final double RPM_b = 1737.0;
    private static final double AIM_kP = 0.03;
    private static final double AIM_kD = 0.002;
    private double lastTx = 0;
    private boolean feedEnabled = false;
    // Start Pose from PP file
    private final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));
    @Override
    public void init() {
        // Hardware mapping
        shootServo       = hardwareMap.servo.get("shootexpservo1");
        hardstopServo    = hardwareMap.servo.get("hardstopServo");
        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor    = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        rollerIntakeMotor= hardwareMap.dcMotor.get("Rollerintakeexp1");
        limelight        = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        // Build PP paths
        buildPaths();
        telemetry.addLine("AUTO READY");
        telemetry.update();
    }
    @Override
    public void start() {
        state = 0;
        scoringPause = false;
        stateTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        if (scoringPause) {
            runRTPauseLogic();
            return;
        }
        runFSM();
        telemetry.addData("State", state);
        telemetry.addData("Pause", scoringPause);
        telemetry.update();
    }
    // ===========================================================
    //                    PATH BUILDING (CLEAN)
    // ===========================================================
    public void buildPaths() {
        // Path 0: Line from (56,8) → (40.8889, 35.9111)
        path[0] = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56, 8, Math.toRadians(90)),
                        new Pose(47.5, 36, Math.toRadians(180))))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();
        // Path 1
        path[1] = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(47.5, 36, Math.toRadians(180)),
                        new Pose(40, 35.7333, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();
        // Path 2
        path[2] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(40, 35.7333, Math.toRadians(180)),
                        new Pose(86.0444, 36.2667, 0),
                        new Pose(49.7778, 99.2, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        // Path 3
        path[3] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.7778, 99.2, Math.toRadians(180)),
                        new Pose(72.1778, 59.9111, 0),
                        new Pose(41.2444, 59.5556, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();
        // Path 4
        path[4] = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(41.2444, 59.5556, Math.toRadians(180)),
                        new Pose(12.8, 59.7333, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();
        // Path 5
        path[5] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(12.8, 59.7333, Math.toRadians(180)),
                        new Pose(36.8, 59.2, 0),
                        new Pose(37.5111, 70.2222, 0),
                        new Pose(11.2, 69.3333, Math.toRadians(180))))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        // Path 6
        path[6] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(11.2, 69.3333, Math.toRadians(180)),
                        new Pose(82.8444, 68.4444, 0),
                        new Pose(49.9556, 99.3778, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        // Path 7
        path[7] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(49.9556, 99.3778, Math.toRadians(180)),
                        new Pose(67.3778, 82.4889, 0),
                        new Pose(33.4222, 83.3778, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();
        // Path 8
        path[8] = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.4222, 83.3778, Math.toRadians(180)),
                        new Pose(12.8, 84.0889, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .build();
        // Path 9
        path[9] = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(12.8, 84.0889, Math.toRadians(180)),
                        new Pose(74.4889, 80, 0),
                        new Pose(49.9556, 99.2, Math.toRadians(180))))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }
    // ===========================================================
    //                     FSM LOGIC
    // ===========================================================
    public void runFSM() {
        if (follower.isBusy()) return;
        switch (state) {
            case 0:
                follower.followPath(path[0], true);
                next();
                break;
            case 1:
                follower.followPath(path[1], true);
                next();
                break;
            // Path 2 → run LT logic
            case 2:
                follower.followPath(path[2], true);
                runLTLogic();
                next();
                break;
            // After Path 3 → RT pause
            case 3:
                follower.followPath(path[3], true);
                startRTPause();
                break;
            case 4:
                follower.followPath(path[4], true);
                next();
                break;
            // Path 5 → LT logic
            case 5:
                follower.followPath(path[5], true);
                runLTLogic();
                next();
                break;
            // After Path 6 → RT pause
            case 6:
                follower.followPath(path[6], true);
                startRTPause();
                break;
            case 7:
                follower.followPath(path[7], true);
                next();
                break;
            // Path 8 → LT logic
            case 8:
                follower.followPath(path[8], true);
                runLTLogic();
                next();
                break;
            // After Path 9 → RT pause
            case 9:
                follower.followPath(path[9], true);
                startRTPause();
                break;
            default:
                break;
        }
    }
    // ===========================================================
    //                  LT LOGIC (Path 2,5,9)
    // ===========================================================
    public void runLTLogic() {
        feedEnabled = false;
        flywheelMotor.setVelocity(0);
        rollerIntakeMotor.setPower(INTAKE_POWER);
        shootrollerServo.setPower(FEED_REVERSE);
        hardstopServo.setPosition(0.75);
    }
    // ===========================================================
    //            RT SCORING PAUSE (after 3,7,10)
    // ===========================================================
    public void startRTPause() {
        scoringPause = true;
        scoreTimer.resetTimer();
    }
    public void runRTPauseLogic() {
        if (scoreTimer.getElapsedTimeSeconds() > 3.0) {
            scoringPause = false;
            feedEnabled = false;
            flywheelMotor.setVelocity(0);
            rollerIntakeMotor.setPower(0);
            shootrollerServo.setPower(0);
            hardstopServo.setPosition(0.75);
            next();
            return;
        }
        // === RT Logic ===
        LLResult ll = limelight.getLatestResult();
        if (ll != null && ll.isValid()) {
            double tx = ll.getTx();
            double ty = ll.getTy();
            double distanceIn = DIST_A / (ty + DIST_B);
            double autoHoodDeg = Range.clip(
                    HOOD_m * distanceIn + HOOD_b,
                    HOOD_MIN_DEG, HOOD_MAX_DEG);
            shootServo.setPosition(degToPos(autoHoodDeg));
            double autoRPM = Math.min(
                    RPM_m * distanceIn + RPM_b,
                    MAX_RPM);
            double targetTPS = rpmToTicksPerSec(autoRPM);
            flywheelMotor.setVelocity(targetTPS);
            double curTPS = Math.abs(flywheelMotor.getVelocity());
            double resumeTPS = rpmToTicksPerSec(autoRPM * RESUME_RPM_FRAC);
            double pauseTPS  = rpmToTicksPerSec(autoRPM * PAUSE_RPM_FRAC);
            if (!feedEnabled && curTPS >= resumeTPS) feedEnabled = true;
            else if (feedEnabled && curTPS < pauseTPS) feedEnabled = false;
            if (feedEnabled) {
                rollerIntakeMotor.setPower(INTAKE_POWER);
                shootrollerServo.setPower(FEED_FORWARD);
            } else {
                rollerIntakeMotor.setPower(0);
                shootrollerServo.setPower(0);
            }
            // PID turn
            double dTx = tx - lastTx;
            lastTx = tx;
            double turnCmd = AIM_kP * tx + AIM_kD * dTx;
            // Hardstop open
            hardstopServo.setPosition(0.15);
        }
    }
    // ===========================================================
    //                     HELPERS
    // ===========================================================
    private void next() {
        state++;
        stateTimer.resetTimer();
    }
    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }
    private double degToPos(double deg) {
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0, 1);
    }
}