
/*
Hello :D my names mikey and i'm the head of software on team 21721. I was looking at the april tag sample code on the PP (pedro pathing) website and it kinda confused me or just wasn't
what I needed to do, so I decided to make my own! Before you worry about the code itself u need to know a bit about April tags. April tags are basically just QR codes; in the sense
that when you scan them they give u a numerical value. the april tag values for this season are as the following-

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/


package org.firstinspires.ftc.teamcode.Archive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
@TeleOp(name="EIAAutoAlignByLimelight", group="Decode2526")
public class EIAAprilTagLimelightMode extends LinearOpMode {
    // -------- Drive Motors --------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    // -------- Limelight --------
    private Limelight3A limelight;
    private static final int    LL_PIPELINE_INDEX = 4;  // choose your pipeline index in Limelight UI
    private static final double AIM_TOL_DEG      = 1.0; // aligned if |tx| <= tolerance
    private static final double MAX_TURN         = 0.6; // clamp PID output to motor power
    // PID gains (tune on-field; start with P-only)
    private static final double kP = 0.03;
    private static final double kI = 0.000;
    private static final double kD = 0.002;
    // Integral hygiene
    private static final double I_ZONE_DEG = 5.0; // only integrate inside this band
    private static final double I_MAX      = 0.2; // clamp integral term
    // PID state
    private double prevError = 0.0;
    private double integral  = 0.0;
    private long   lastTsNanos = 0L;
    @Override
    public void runOpMode() {
        // ---- Map drive motors (uses your existing names) ----
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // Optional: brake when zero power
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // ---- Limelight init ----
        limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(LL_PIPELINE_INDEX);
        limelight.start();
        telemetry.addLine("TeleOp_LimelightAlignDrive READY (LB = Align)");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        lastTsNanos = System.nanoTime();
        while (opModeIsActive()) {
            // -------- Driver inputs --------
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  =  gamepad1.left_stick_x * 1.3; // strafe
            double rx =  gamepad1.right_stick_x;  // rotate
            boolean alignHold = gamepad1.left_bumper;
            double alignOut = 0.0;
            boolean hasTarget = false;
            double txDeg = 0.0;
            if (alignHold) {
                LLResult ll = limelight.getLatestResult();
                if (ll != null && ll.isValid()) {
                    hasTarget = true;
                    txDeg = ll.getTx();           // horizontal error in degrees
                    alignOut = pidTurnFromTx(txDeg);
                    rx = alignOut;                // override driver turn while aligning
                } else {
                    // No target: don't spin blindly; reset PID
                    resetPid();
                }
            } else {
                resetPid();
            }
            // -------- Mecanum drive mix --------
            double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            frontLeftMotor.setPower((y + x + rx) / denom);
            backLeftMotor.setPower( (y - x + rx) / denom);
            frontRightMotor.setPower((y - x - rx) / denom);
            backRightMotor.setPower( (y + x - rx) / denom);
            // -------- Telemetry --------
            telemetry.addLine("---- Drive + Align ----");
            telemetry.addData("Align (hold LB)", alignHold);
            telemetry.addData("Has Target", hasTarget);
            telemetry.addData("tx (deg)", "%.2f", txDeg);
            telemetry.addData("PID Out", "%.3f", alignOut);
            telemetry.addData("Aligned?", Math.abs(txDeg) <= AIM_TOL_DEG);
            telemetry.update();
        }
    }
    // ======= Limelight PID helper =======
    private double pidTurnFromTx(double txDeg) {
        long now = System.nanoTime();
        double dt = (now - lastTsNanos) / 1e9;
        if (dt <= 0) dt = 1e-3;
        lastTsNanos = now;
        double error = txDeg; // target is 0 deg
        // Integral (only inside the I-zone to avoid windup)
        if (Math.abs(error) < I_ZONE_DEG) {
            integral += error * dt;
            integral = Range.clip(integral, -I_MAX, I_MAX);
        } else {
            integral = 0.0;
        }
        double derivative = (error - prevError) / dt;
        prevError = error;
        double out = kP * error + kI * integral + kD * derivative;
        // Snap to zero inside tolerance
        if (Math.abs(error) <= AIM_TOL_DEG) {
            out = 0.0;
        }
        return Range.clip(out, -MAX_TURN, MAX_TURN);
    }
    private void resetPid() {
        prevError = 0.0;
        integral  = 0.0;
        lastTsNanos = System.nanoTime();
    }
}