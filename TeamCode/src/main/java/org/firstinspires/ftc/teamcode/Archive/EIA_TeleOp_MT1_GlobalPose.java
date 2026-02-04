package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp(name = "EIA_TeleOp_MT1_GlobalPose", group = "Vision")
@Disabled
public class EIA_TeleOp_MT1_GlobalPose extends OpMode {
    private Follower follower;
    private Limelight3A limelight;
    private final EIA_Mt1PoseSolver mt1 = new EIA_Mt1PoseSolver();
    // ---------------------------
    // Tag 20 location (meters)
    // ---------------------------
    private static final Pose TAG20 = new Pose(1.143, 1.714, Math.toRadians(90));
    // ---------------------------
    // LL TX PID
    // ---------------------------
    private double kP = 0.03;
    private double kD = 0.002;
    private double prevError = 0;
    private long lastTime = System.nanoTime();
    private static final double DEADZONE = 0.2;
    private static final double MAX_TURN = 0.6;
    // ---------------------------
    //  Motors
    // ---------------------------
    private com.qualcomm.robotcore.hardware.DcMotor frontLeftMotor, backLeftMotor;
    private com.qualcomm.robotcore.hardware.DcMotor frontRightMotor, backRightMotor;
    @Override
    public void init() {
        // Pedro follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.update();
        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);
        // Motors
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");
        frontRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void start() {
        limelight.start();
    }
    @Override
    public void loop() {
        // ----------------------------
        // Pedro Odometry
        // ----------------------------
        follower.update();
        Pose odomPose = follower.getPose();
        // ----------------------------
        // Read Limelight
        // ----------------------------
        LLResult result = limelight.getLatestResult();
        Pose visionPose = null;
        if (result != null && result.isValid() && result.getTa() > 0.00001) {
            // Always solve for TAG20 (your requirement)
            visionPose = mt1.solvePoseFromLL(
                    result.getTx(),
                    result.getTy(),
                    result.getTa(),
                    TAG20
            );
        }
        // ----------------------------
        // Fuse Odometry + Vision
        // ----------------------------
        Pose globalPose;
        if (visionPose != null) {
            double a = 0.40;
            double x = odomPose.getX() * (1 - a) + visionPose.getX() * a;
            double y = odomPose.getY() * (1 - a) + visionPose.getY() * a;
            double hO = odomPose.getHeading();
            double hV = visionPose.getHeading();
            double dh = wrap(hV - hO);
            double h = wrap(hO + a * dh);
            globalPose = new Pose(x, y, h);
        } else {
            globalPose = odomPose;
        }
        follower.setPose(globalPose);
        // ----------------------------
        // Driver Controls
        // ----------------------------
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x * 1.3;
        double rx =  gamepad1.right_stick_x;
        // ----------------------------
        // Auto Align (hold X)
        // ----------------------------
        if (gamepad1.x &&
                result != null &&
                result.isValid() &&
                Math.abs(result.getTx()) > 0.05)
        {
            double tx = result.getTx();
            rx = pidTx(tx);
        }
        // ----------------------------
        // Your Known-Good Mecanum Drive
        // ----------------------------
        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        frontLeftMotor.setPower((y + x + rx) / denom);
        backLeftMotor.setPower( (y - x + rx) / denom);
        frontRightMotor.setPower((y - x - rx) / denom);
        backRightMotor.setPower( (y + x - rx) / denom);
        // ----------------------------
        // TELEMETRY
        // ----------------------------
        telemetry.addLine("=== GLOBAL POSE ===");
        telemetry.addData("X", "%.3f", globalPose.getX());
        telemetry.addData("Y", "%.3f", globalPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(globalPose.getHeading()));
        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("tx", result != null ? result.getTx() : 0);
        telemetry.addData("ty", result != null ? result.getTy() : 0);
        telemetry.addData("ta", result != null ? result.getTa() : 0);
        telemetry.addData("Vision Valid?", visionPose != null);
        telemetry.update();
    }
    // ----------------------------
    // PID for TX Alignment
    // ----------------------------
    private double pidTx(double errorDeg) {
        if (Math.abs(errorDeg) < DEADZONE) return 0;
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;
        double derivative = (errorDeg - prevError) / dt;
        prevError = errorDeg;
        double output = kP * errorDeg + kD * derivative;
        return clamp(output, -MAX_TURN, MAX_TURN);
    }
    // ----------------------------
    // Helpers
    // ----------------------------
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
    private static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}