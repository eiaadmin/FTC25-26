/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/*
 * This Auto shoots from the launch area, fetches the first row of balls, comes back to the
 * launch area and shoots it. Then it gets the balls from the human player zone and comes back
 * to the launch area and shoots it.
 */

@Autonomous(name = "EIA9BallBlueWallFarAuto", group = "Decode2526")

public class EIA9BallBlueWallFarAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pathState;
    private final Pose startPose = new Pose(65, 7.625,Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 19, Math.toRadians(291));
    private final Pose pickup1Pose = new Pose(57.15, 27.5,Math.toRadians(-180));
    private final Pose pickup1grabPose = new Pose(21, 27.5,Math.toRadians(-180));
    private final Pose pickup1scorePose = new Pose(55, 19, Math.toRadians(291));
    private final Pose pickup2Pose = new Pose(36.05, 18.84,Math.toRadians(-180));
    private final Pose pickup2grabPose = new Pose(21, 2,Math.toRadians(-180));
    private final Pose pickup2Pose2 = new Pose(35, 5,Math.toRadians(-180));
    private final Pose pickup2grabPose2 = new Pose(21, 1,Math.toRadians(-180));
    private final Pose pickup2Pose3 = new Pose(35, 4,Math.toRadians(-180));
    private final Pose pickup2grabPose3 = new Pose(21, -5,Math.toRadians(-180));
    private final Pose pickup2grabPose4 = new Pose(30, 15,Math.toRadians(-180));
    private final Pose scorePose2 = new Pose(55, 19, Math.toRadians(291));
    private final Pose pickup3Pose = new Pose(25, 15,Math.toRadians(-180));
    private final Pose pickup3grabPose = new Pose(25, 10,Math.toRadians(-180));
    private final Pose scorePose3 = new Pose(55, 19, Math.toRadians(291));
    private final Pose leavePose = new Pose(65, 30,Math.toRadians(-180));
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2,landingPath,grabPickup3, scorePickup3;;//, grabPickup3, scorePickup3, landingPath;


    // -------- Mechanisms --------
    private DcMotorEx flywheelMotor;      // velocity control
    private DcMotorEx flywheelMotor1;      // velocity control
    private DcMotor rollerIntakeMotor,rollerIntakeMotor2;
    private CRServo shootrollerServo;   // feeder (CR)
    private Servo shootServo,hardstopServo;        // hood (positional)
    //private Limelight3A limelight;
    // hood (positional)

    // -------- Hood mapping + presets (tune these) --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40;

    private double PRESET_HIGH_DEG = 38;

    // -------- Flywheel velocity control --------
    private static final double TICKS_PER_REV = 28.0;  // from your motor specs
    private static final double GEAR_RATIO    = 1.0;   // motor revs per flywheel rev

    // RPM targets
    private static final double TARGET_RPM    = 3950;//4500;//4500.0; // as requested
    private static final double IDLE_RPM       = 3900;
    double targetTPS = 0;

    // Feeding thresholds (hysteresis)
    private static final double RESUME_RPM_FRAC = 0.93; // resume feed at >= 85% of target
    private static final double PAUSE_RPM_FRAC  = 0.88; // pause feed if < 80% of target

    // Derived ticks/sec thresholds
    private static final double TARGET_TPS = rpmToTicksPerSec(TARGET_RPM);
    private static final double RESUME_TPS = rpmToTicksPerSec(TARGET_RPM * RESUME_RPM_FRAC);
    private static final double PAUSE_TPS  = rpmToTicksPerSec(TARGET_RPM * PAUSE_RPM_FRAC);
    private double lastAppliedTPS;

    // -------- Intake/feeder powers --------
    private static final double INTAKE_POWER = 1.0, INTAKE_POWER_PPG=0.8;
    private static final double FEED_FORWARD = -1.0; // forward
    private static final double FEED_REVERSE = +1.0; // reverse

    private static final double FW_kP=440,FW_kI=0.0,FW_kD=0.0;

    // State: feeding allowed while RT is held
    private boolean feedEnabled = false;

    private static double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV * GEAR_RATIO;
    }
    private static double ticksPerSecToRpm(double tps) {
        double mechTpr = TICKS_PER_REV * GEAR_RATIO;
        return (tps / mechTpr) * 60.0;
    }
    private double degToPos(double deg){
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, 0.0, 1.0);
    }
    public void intakeArtifacts() {

        targetTPS = rpmToTicksPerSec(IDLE_RPM);
        flywheelMotor.setVelocity(targetTPS);

        double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(TARGET_RPM), 0.0, 1.0);
        flywheelMotor1.setPower(approxPower);
        rollerIntakeMotor.setPower(INTAKE_POWER);
        rollerIntakeMotor2.setPower(INTAKE_POWER);
        shootrollerServo.setPower(FEED_REVERSE);
        hardstopServo.setPosition(0.55);

        if(pathTimer.getElapsedTimeSeconds() > 3.15 && pathState==-2){
            setPathState(3);
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.15 && pathState==-4){
            setPathState(6);
        }
       /* if(pathTimer.getElapsedTimeSeconds() > 2.65 && pathState==-8){
            setPathState(9);
        }*/

    }
    public void enableShooter() {

        telemetry.addData("Shooter position",degToPos(PRESET_HIGH_DEG));
        telemetry.update();
        shootServo.setPosition(degToPos(PRESET_HIGH_DEG));

        // Command target velocity
        hardstopServo.setPosition(0.15);
        // Measure current speed
        double tps = Math.abs(flywheelMotor.getVelocity());

        telemetry.addData("Flywheel Velocity",tps);
        telemetry.update();
        // Hysteresis:
        // - If currently NOT feeding, enable once we cross RESUME_TPS.
        // - If currently feeding, pause if we dip below PAUSE_TPS.
        if (!feedEnabled && tps >= RESUME_TPS) {
            feedEnabled = true;     // first time at speed -> start/continue feeding
        } else if (feedEnabled && tps < PAUSE_TPS) {
            feedEnabled = false;    // dip detected -> pause feeding until back up to RESUME_TPS
            targetTPS = rpmToTicksPerSec(IDLE_RPM);
            flywheelMotor.setVelocity(targetTPS);

            double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(TARGET_RPM), 0.0, 1.0);
            flywheelMotor1.setPower(approxPower);
        }
        if (feedEnabled) {
            rollerIntakeMotor.setPower(INTAKE_POWER);
            rollerIntakeMotor2.setPower(INTAKE_POWER);
            shootrollerServo.setPower(FEED_FORWARD);
        } else {
            rollerIntakeMotor.setPower(0);
            rollerIntakeMotor2.setPower(0);
            shootrollerServo.setPower(0.0);
        }
        if(pathTimer.getElapsedTimeSeconds() > 4.25 && pathState ==-1) {
            setPathState(2);
            feedEnabled = false;
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.25 && pathState ==-3) {
            setPathState(5);
            feedEnabled = false;
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.25 && pathState ==-6) {
            setPathState(8);
            feedEnabled = false;
        }
        /*if(pathTimer.getElapsedTimeSeconds() > 1.5 && pathState ==-9) {
            setPathState(11);
            feedEnabled = false;
        }*/

        telemetry.addData("feedEnabled ", feedEnabled);
        telemetry.addData("Enable Shooter Elapsed Time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup1Pose,  pickup1grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose,  pickup1scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(291))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1scorePose,  pickup2Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2Pose,  pickup2grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2grabPose,  pickup2Pose2))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2Pose2,  pickup2grabPose2))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2grabPose2,  pickup2Pose3))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2Pose3,  pickup2grabPose3))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2Pose3,  pickup2grabPose3))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup2grabPose3,  pickup2grabPose4))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2grabPose4,  scorePose2))
                .setConstantHeadingInterpolation(Math.toRadians(291))
                .build();

        /*grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2,  pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup3Pose,  pickup3grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3grabPose,  scorePose3))
                .setConstantHeadingInterpolation(Math.toRadians(291))
                .build();*/

        landingPath = follower.pathBuilder()
                .addPath(new BezierLine(scorePose3,  leavePose))
                .setConstantHeadingInterpolation(Math.toRadians(245))
                .build();
    }

    public void autonomousPathUpdate(){
        if (follower.isBusy()) return;
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                follower.setMaxPower(1.0);
                flywheelMotor.setVelocity(TARGET_TPS);
                setPathState(1);
                break;
            case 1:
                enableShooter();
                setPathState(-1);
                break;
            case 2:
                follower.followPath(grabPickup1,true);
                follower.setMaxPower(0.85);
                intakeArtifacts();
                setPathState(-2);
                break;
            case 3:
                //rollerIntakeMotor.setPower(INTAKE_POWER_PPG);
                follower.followPath(scorePickup1, true);
                follower.setMaxPower(1.0);
                setPathState(4);
                break;
            case 4:
                enableShooter();
                setPathState(-3);
                break;
            case 5:
                follower.followPath(grabPickup2,true);
                follower.setMaxPower(0.65);
                intakeArtifacts();
                setPathState(-4);
                break;
            case 6:
                follower.followPath(scorePickup2, true);
                follower.setMaxPower(1.0);
                setPathState(7);
                break;
            case 7:
                enableShooter();
                setPathState(-6);
                break;
            case 8:
                /*follower.followPath(grabPickup3,true);
                follower.setMaxPower(0.75);
                intakeArtifacts();
                setPathState(-8);
                break;
            case 9:
                follower.followPath(scorePickup3, true);
                follower.setMaxPower(1.0);
                setPathState(10);
                break;
            case 10:
                enableShooter();
                setPathState(-9);
                break;
            case 11:*/
                flywheelMotor.setVelocity(0.0);
                flywheelMotor1.setVelocity(0.0);
                shootrollerServo.setPower(FEED_REVERSE);
                follower.followPath(landingPath, true);
                follower.setMaxPower(1.0);
                setPathState(-11);
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        if (TARGET_TPS > 1 && Math.abs(TARGET_TPS-lastAppliedTPS)>25){
            double kF = 19;
            PIDFCoefficients flywhlpidf = new PIDFCoefficients(FW_kP,FW_kI,FW_kD,kF);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywhlpidf);
            lastAppliedTPS=TARGET_TPS;
        }
        targetTPS = rpmToTicksPerSec(TARGET_RPM);
        flywheelMotor.setVelocity(targetTPS);

        double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(TARGET_RPM), 0.0, 1.0);
        flywheelMotor1.setPower(approxPower);

        if (pathState == -1 || pathState == -3 || pathState == -6 || pathState == -9) {
            enableShooter();
        }else if (pathState == -2 || pathState == -4 || pathState == -8 ){
            intakeArtifacts();
        }else {
            autonomousPathUpdate();
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state ", pathState);
        telemetry.addData("x ", follower.getPose().getX());
        telemetry.addData("y ", follower.getPose().getY());
        telemetry.addData("heading ", follower.getPose().getHeading());
        telemetry.addData("TargetTPS ", TARGET_TPS);
        telemetry.addData("lastAppliedTPS ", lastAppliedTPS);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        shootServo = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);

        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");
        rollerIntakeMotor2 = hardwareMap.dcMotor.get("Rollerintakeexp2");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor1.setDirection(DcMotor.Direction.REVERSE);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        hardstopServo    = hardwareMap.servo.get("hardstopServo");
        hardstopServo.setPosition(0.55);

        buildPaths();
        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        targetTPS = rpmToTicksPerSec(TARGET_RPM);
        flywheelMotor.setVelocity(targetTPS);

        double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(TARGET_RPM), 0.0, 1.0);
        flywheelMotor1.setPower(approxPower);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}



