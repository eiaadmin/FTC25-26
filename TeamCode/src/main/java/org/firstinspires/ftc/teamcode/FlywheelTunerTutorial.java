package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTunerTutorial extends OpMode {
    public DcMotorEx flywheelMotor1;
    public DcMotorEx flywheelMotor2;

    // These are TRUE RPM targets now
    public double highVelocity = 3950;
    public double lowVelocity  = 3100;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0 , 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    // Encoder constants
    private static final double TICKS_PER_REV = 28.0;

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    private double ticksPerSecondToRpm(double tps) {
        return (tps * 60.0) / TICKS_PER_REV;
    }

    @Override
    public void init(){
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");

        flywheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor2: follower open-loop (safe even if no encoder)
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P , 0, 0, F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop(){

        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()){
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P , 0, 0, F);
        flywheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // Convert RPM target -> ticks/sec for the SDK call
        double targetTPS = rpmToTicksPerSecond(curTargetVelocity);

        flywheelMotor1.setVelocity(targetTPS);
        flywheelMotor2.setPower(flywheelMotor1.getPower());

        double curTPS = flywheelMotor1.getVelocity();
        double curRPM = ticksPerSecondToRpm(curTPS);

        double errorRPM = curTargetVelocity - curRPM;

        telemetry.addData("Target RPM", curTargetVelocity);
        telemetry.addData("Current RPM", "%.2f", curRPM);
        telemetry.addData("Error RPM", "%.2f", errorRPM);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f", P);
        telemetry.addData("Tuning F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Target TPS", "%.2f", targetTPS);
        telemetry.addData("Current TPS", "%.2f", curTPS);
    }
}
