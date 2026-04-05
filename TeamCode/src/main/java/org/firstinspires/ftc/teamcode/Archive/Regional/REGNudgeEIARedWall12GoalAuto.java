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
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name = "REGNudgeEIARedWall12Auto", group = "Decode2526")
@Disabled
public class REGNudgeEIARedWall12GoalAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pathState;
    private final Pose startPose = new Pose(79, 7.625,Math.toRadians(270)); // Start Pose of our robot.
    private final Pose nudgePose = new Pose(90, 7.625,Math.toRadians(270));
    private final Pose scorePose1 = new Pose(89, 17, Math.toRadians(245));
    private final Pose scorePose = new Pose(88.4, 75.1, Math.toRadians(-135));//93.4, 83.57, Math.toRadians(-135));
    private final Pose scoreControlPose = new Pose(79.48, 45.06, Math.toRadians(-135));
    private final Pose pickup1Pose =new Pose(82.75,72, Math.toRadians(0)); // Grab Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup1grabPose = new Pose(110, 72, Math.toRadians(0));
    private final Pose releasegatePose = new Pose(102, 65, Math.toRadians(0)); // Score Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose releasegateEnablePose = new Pose(125,65, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(83.4, 80.1, Math.toRadians(-135)); // Score Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2ControlPose = new Pose(66, 65, Math.toRadians(-135));
    private final Pose pickup3Pose = new Pose(82.75, 46.5, Math.toRadians(0));
    private final Pose pickup3grabPose = new Pose(120, 46.5, Math.toRadians(0));//53.67
    private final Pose pickup4Pose = new Pose(83.4, 80.1, Math.toRadians(-135)); // Score Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup4ControlPose = new Pose(66, 65, Math.toRadians(-135));
    private final Pose pickup5Pose = new Pose(79.5, 27,Math.toRadians(0));
    private final Pose pickup5grabPose = new Pose(125.7, 27,Math.toRadians(0));
    //129.87, 85.21
    private final Pose pickup6Pose = new Pose(88.4, 75.1, Math.toRadians(-135));//93.4, 83.57, Math.toRadians(-135)); // Score Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup6ControlPose = new Pose(79.48, 45.06, Math.toRadians(-135));
    private final Pose landingPose = new Pose(100, 65, Math.toRadians(0));
    private PathChain scorePreload,grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, landingPath;

    // -------- Mechanisms --------
    private DcMotorEx flywheelMotor, flywheelMotor1;      // velocity control
    private DcMotor rollerIntakeMotor, rollerIntakeMotor2;
    private CRServo shootrollerServo;   // feeder (CR)
    private Servo shootServo,hardstopServo;         // hood (positional)

    // -------- Hood mapping + presets (tune these) --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;

    private double PRESET_HIGH_DEG = 35;

    // -------- Flywheel velocity control --------
    private static final double TICKS_PER_REV = 28.0;  // from your motor specs
    private static final double GEAR_RATIO    = 1.0;   // motor revs per flywheel rev

    // RPM targets
    private static final double TARGET_RPM    = 3200;//4500;//4500.0; // as requested
    private static final double IDLE_RPM       = 3150;
    double targetTPS = 0;

    private double lastAppliedTPS;
    private static final double FW_kP=440,FW_kI=0.0,FW_kD=0.0;//310//FW_kP=8.5,FW_kI=0.0,FW_kD=0.0;

    // Feeding thresholds (hysteresis)
    private static final double RESUME_RPM_FRAC = 0.90; // resume feed at >= 85% of target
    private static final double PAUSE_RPM_FRAC  = 0.85; // pause feed if < 80% of target

    // Derived ticks/sec thresholds
    private static final double TARGET_TPS = rpmToTicksPerSec(TARGET_RPM);
    private static final double RESUME_TPS = rpmToTicksPerSec(TARGET_RPM * RESUME_RPM_FRAC);
    private static final double PAUSE_TPS  = rpmToTicksPerSec(TARGET_RPM * PAUSE_RPM_FRAC);

    // -------- Intake/feeder powers --------
    private static final double INTAKE_POWER = 1.0, INTAKE_POWER_PPG=0.8;
    private static final double FEED_FORWARD = -1.0; // forward
    private static final double FEED_REVERSE = +1.0; // reverse

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

        //flywheelMotor.setVelocity(0.0);
        //flywheelMotor1.setVelocity(0.0);
        targetTPS = rpmToTicksPerSec(IDLE_RPM);
        flywheelMotor.setVelocity(targetTPS);

        double approxPower = Range.clip(targetTPS / rpmToTicksPerSec(TARGET_RPM), 0.0, 1.0);
        flywheelMotor1.setPower(approxPower);
        rollerIntakeMotor.setPower(INTAKE_POWER_PPG);
        rollerIntakeMotor2.setPower(INTAKE_POWER_PPG);
        shootrollerServo.setPower(FEED_REVERSE);
        hardstopServo.setPosition(0.40);

        if(pathTimer.getElapsedTimeSeconds() > 3.90 && pathState==-2){
            setPathState(3);
        }
        if(pathTimer.getElapsedTimeSeconds() > 3.45 && pathState==-4){
            setPathState(6);
        }
        if(pathTimer.getElapsedTimeSeconds() >3.65 && pathState==-7){
            setPathState(9);
        }
    }
    public void enableShooter() {

        shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
        hardstopServo.setPosition(0.15);
        // Measure current speed
        double tps = Math.abs(flywheelMotor.getVelocity());

        // Hysteresis:
        // - If currently NOT feeding, enable once we cross RESUME_TPS.
        // - If currently feeding, pause if we dip below PAUSE_TPS.
        if (!feedEnabled && tps >= RESUME_TPS) {
            feedEnabled = true;     // first time at speed -> start/continue feeding
        } else if (feedEnabled && tps < PAUSE_TPS) {
            feedEnabled = false;    // dip detected -> pause feeding until back up to RESUME_TPS
        }
        if (feedEnabled) {
            rollerIntakeMotor.setPower(INTAKE_POWER);
            rollerIntakeMotor2.setPower(INTAKE_POWER);
            shootrollerServo.setPower(FEED_FORWARD);
        } else {
            rollerIntakeMotor.setPower(0.0);
            rollerIntakeMotor2.setPower(0.0);
            shootrollerServo.setPower(0.0);
        }

        if(pathTimer.getElapsedTimeSeconds() > 2.05 && pathState ==-1) {
            setPathState(2);
            feedEnabled = false;
        }
        if(pathTimer.getElapsedTimeSeconds() > 2.05 && pathState ==-3) {
            setPathState(5);
            feedEnabled = false;
        }
        if(pathTimer.getElapsedTimeSeconds() > 2.05 && pathState ==-6) {
            setPathState(8);
            feedEnabled = false;
        }
        if(pathTimer.getElapsedTimeSeconds() > 2.05 && pathState ==-9) {
            setPathState(11);
            feedEnabled = false;
        }
        telemetry.addData("Enable Shooter Elapsed Time: ", pathTimer.getElapsedTimeSeconds());
        telemetry.update();
    }
    public void buildPaths() {
        scorePreload =  follower.pathBuilder()
                .addPath(new BezierLine(startPose,  nudgePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), nudgePose.getHeading())
                .addPath(new BezierLine(nudgePose,  scorePose1))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .addPath(new BezierCurve(scorePose1,  scoreControlPose, scorePose))
                .setLinearHeadingInterpolation(scorePose1.getHeading(), scorePose.getHeading())
                .build();
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,  pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickup1Pose,  pickup1grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickup1grabPose,  releasegatePose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(releasegatePose,  releasegateEnablePose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(releasegateEnablePose,  pickup2ControlPose, pickup2Pose))
                .setLinearHeadingInterpolation(releasegateEnablePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose,  pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickup3Pose,  pickup3grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose,  pickup4ControlPose, pickup4Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup4Pose.getHeading())
                .build();
        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose,  pickup5Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(pickup5Pose,  pickup5grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup5grabPose,  pickup6ControlPose, pickup6Pose))
                .setLinearHeadingInterpolation(pickup5grabPose.getHeading(), pickup6Pose.getHeading())
                .build();
        landingPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup6Pose, landingPose)))
                .setLinearHeadingInterpolation(pickup6Pose.getHeading(), landingPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
                flywheelMotor.setVelocity(0.0);
                flywheelMotor1.setVelocity(0.0);
                follower.followPath(scorePreload);
                follower.setMaxPower(1.0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() ) {
                    enableShooter();
                    setPathState(-1);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() ) {
                    telemetry.addData("Inside first pickup",PRESET_HIGH_DEG);
                    telemetry.update();
                    follower.followPath(grabPickup1,true);
                    follower.setMaxPower(0.85);
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    intakeArtifacts();
                    setPathState(-2);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                /* Grab Sample */
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup1, true);
                follower.setMaxPower(1.0);
                setPathState(4);
                break;
            case 4:
                if(!follower.isBusy() ) {
                    enableShooter();
                    setPathState(-3);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() ) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2,true);
                    follower.setMaxPower(0.85);
                    intakeArtifacts();
                    setPathState(-4);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                /* Grab Sample */

                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup2, true);
                follower.setMaxPower(1.0);
                setPathState(7);
                break;
            case 7:
                if(!follower.isBusy() ) {
                    enableShooter();
                    setPathState(-6);
                }
                break;
            case 8:

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy() ) {
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup3,true);
                    follower.setMaxPower(0.85);
                    intakeArtifacts();
                    setPathState(-7);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                /* Grab Sample */
                /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(scorePickup3, true);
                follower.setMaxPower(1.0);
                setPathState(10);
                break;
            case 10:
                if(!follower.isBusy() ) {
                    enableShooter();
                    setPathState(-9);
                }
                break;
            case 11:
                flywheelMotor.setVelocity(0.0);
                flywheelMotor1.setVelocity(0.0);
                shootrollerServo.setPower(0.0);
                rollerIntakeMotor.setPower(0.0);
                rollerIntakeMotor2.setPower(0.0);
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
            double kF = 19.0;//14.9;
            PIDFCoefficients flywhlpidf = new PIDFCoefficients(FW_kP,FW_kI,FW_kD,kF);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,flywhlpidf);
            lastAppliedTPS=TARGET_TPS;
        }

        flywheelMotor.setVelocity(TARGET_TPS);
        double approxPower = Range.clip(TARGET_TPS / rpmToTicksPerSec(TARGET_RPM), 0,1);
        flywheelMotor1.setPower(approxPower);
        if (pathState == -1 || pathState == -3 || pathState == -6 || pathState == -9) {
            enableShooter();
        }else if (pathState == -2 || pathState == -4 || pathState == -7){
            intakeArtifacts();
        }
        else {
            autonomousPathUpdate();
        }

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
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
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor1.setDirection(DcMotor.Direction.REVERSE);
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");
        rollerIntakeMotor2 = hardwareMap.dcMotor.get("Rollerintakeexp2");
        hardstopServo    = hardwareMap.servo.get("hardstopServo");
        //hardstopServo.setPosition(0.55);

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor2.setDirection(DcMotor.Direction.REVERSE);
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
        flywheelMotor.setVelocity(rpmToTicksPerSec(TARGET_RPM));
        shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
