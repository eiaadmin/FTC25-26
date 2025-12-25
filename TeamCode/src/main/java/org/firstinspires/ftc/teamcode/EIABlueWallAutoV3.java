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
import com.qualcomm.hardware.limelightvision.LLResult;
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

@Autonomous(name = "EIABlueWallAutoV3", group = "Decode2526")

public class EIABlueWallAutoV3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pathState;
    private boolean rtPauseActive = false;
    private Timer rtTimer = new Timer();
    private boolean ltActive = false;
    private static final double MAX_TURN   = 0.6;
    private final Pose startPose = new Pose(65, 7.625, Math.toRadians(270)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(55, 19, Math.toRadians(291)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose pickup1Pose = new Pose(40, 36,Math.toRadians(288));
    private final Pose pickup1ControlPose = new Pose(55, 38, Math.toRadians(180));
    private final Pose pickup1grabPose = new Pose(20, 36,Math.toRadians(0));
    private final Pose pickup1scorePose = new Pose(55, 19, Math.toRadians(291));

    //private final Pose pickup1ControlPose = new Pose(62.88, 17.21, Math.toRadians(0));
    /*private final Pose pickup2Pose = new Pose(59.8, 83.57, Math.toRadians(-45)); // Score Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2ControlPose = new Pose(70.26,31.54, Math.toRadians(-45));

    private final Pose pickup3Pose = new Pose(58.38, 48.4, Math.toRadians(-180)); //89.51, 52.44

    //private final Pose pickup3Pose = new Pose(58.99, 54.07, Math.toRadians(-180));
    private final Pose pickup3grabPose = new Pose(17, 48.4, Math.toRadians(-180));
    private final Pose pickup4Pose = new Pose(59.8, 83.57, Math.toRadians(-43)); // Score Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup4ControlPose = new Pose(70.26,31.54, Math.toRadians(-45));
    //private final Pose pickup5Pose = new Pose(126.59, 84.19, Math.toRadians(0)); // Grab Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup5Pose = new Pose(60.43, 70.67, Math.toRadians(-180));
    private final Pose pickup5grabPose = new Pose(17, 70.67, Math.toRadians(-180)); //18.44, 77.02

    //private final Pose pickup5ControlPose = new Pose(41.17, 91.56, Math.toRadians(0));
//129.87, 85.21
    private final Pose pickup6Pose = new Pose(59.8, 83.57, Math.toRadians(-45)); // Score Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup6ControlPose = new Pose(61.4, 78.2, Math.toRadians(-45));
    private final Pose landingPose = new Pose(47.7, 64.6, Math.toRadians(-45)); // Landing Pose of our robot. It is facing the goal at a 135 degree angle.

    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, landingPath;
*/
    private PathChain scorePreload,grabPickup1Path1,grabPickup1Path2, scorePickup1;
    //private Path scorePreload;
    // -------- Mechanisms --------
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotorEx flywheelMotor,flywheelMotor2;      // velocity control
    private DcMotor rollerIntakeMotor;
    private CRServo shootrollerServo;   // feeder (CR)
    private Servo shootServo,hardstopServo;        // hood (positional)
    //private Limelight3A limelight;
    // hood (positional)

    // -------- Hood mapping + presets (tune these) --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40;

    private double PRESET_HIGH_DEG = 38,lastAppliedTPS = -1,activeShotRPM = 0.0,activeShotHoodDeg = 0.0;

    // -------- Flywheel velocity control --------
    private static final double TICKS_PER_REV = 28.0;  // from your motor specs
    private static final double GEAR_RATIO    = 1.0;   // motor revs per flywheel rev
    private static final double X_OFFSET = +6.0;
    private static final double Y_OFFSET = -12.0;
    // RPM targets
    private static final double TARGET_RPM    = 4500.0;//4500;//4500.0; // as requested
    private static final double MAX_RPM       = 4500.0;

    private static final double SHOT_A_RPM = 4500.0;
    private static final double SHOT_A_HOOD_DEG = 33.0;

    // After Path 8, 11, 14 -> 4100 RPM, 27 deg
    private static final double SHOT_B_RPM = 4100.0;
    private static final double SHOT_B_HOOD_DEG = 27.0;

    // Feeding thresholds (hysteresis)
    private static final double RESUME_RPM_FRAC = 0.85; // resume feed at >= 85% of target
    private static final double PAUSE_RPM_FRAC  = 0.80; // pause feed if < 80% of target

    // Derived ticks/sec thresholds
    private static final double TARGET_TPS = rpmToTicksPerSec(TARGET_RPM);
    private static final double RESUME_TPS = rpmToTicksPerSec(TARGET_RPM * RESUME_RPM_FRAC);
    private static final double PAUSE_TPS  = rpmToTicksPerSec(TARGET_RPM * PAUSE_RPM_FRAC);
    private static final double FW_kP = 0.01;
    private static final double FW_kI = 0.0;
    private static final double FW_kD = 0.0;

    private double prevError = 0.0;
    private double integral  = 0.0;
    private long   lastTsNanos = 0L;
    private static final double RT_PAUSE_SECONDS = 3;

    // -------- Intake/feeder powers --------
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0; // forward
    private static final double FEED_REVERSE = +1.0; // reverse

    // State: feeding allowed while RT is held
    private boolean feedEnabled = false;

    private static Pose offXY(Pose s) {
        if (Math.abs(s.getX() - 55.0) < 1e-9 && Math.abs(s.getY() - 19.0) < 1e-9) {
            return s;
        } else if (Math.abs(s.getX() - 19.0) < 1e-9 && Math.abs(s.getY() - 13.0) < 1e-9) {
            return new Pose(s.getX() + X_OFFSET, s.getY()); // your special-case rule
        } else {
            return new Pose(s.getX() + X_OFFSET, s.getY() + Y_OFFSET);
        }
    }

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
    private boolean isIntakePathState(int state) {
        return state == 3   // Path3 running
                || state == 8   // Path6 running
                || state == 10  // Path10 running
                || state == 13  // Path13 running
                || state == 14; // Path14 running
    }
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
    private void resetAimPid() {
        prevError = 0.0;
        integral  = 0.0;
        lastTsNanos = System.nanoTime();
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

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(offXY(startPose), offXY(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(291))
                .build();

        /* This is our grabPickup1 PathChain. */
        grabPickup1Path1 = follower.pathBuilder()
                .addPath(new BezierCurve(offXY(scorePose), offXY(pickup1ControlPose), offXY(pickup1Pose)))
                .setLinearHeadingInterpolation(Math.toRadians(288), Math.toRadians(180))
                .build();

        grabPickup1Path2 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(pickup1Pose), offXY(pickup1grabPose)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(pickup1grabPose), offXY(pickup1scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(288))
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*grabPickup2 = follower.pathBuilder()
                //.addPath(new BezierCurve(pickup2Pose,  pickup3ControlPose, pickup3Pose))
                //.setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
                .addPath(new BezierLine(pickup2Pose,  pickup3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup3Pose,  pickup3grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();*/

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose,  pickup4ControlPose, pickup4Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), pickup4Pose.getHeading())
                .build();*/

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose,  pickup5Pose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .addPath(new BezierLine(pickup5Pose,  pickup5grabPose))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                //.addPath(new BezierCurve(pickup4Pose,  pickup5ControlPose, pickup5Pose))
                //.setLinearHeadingInterpolation(pickup4Pose.getHeading(), pickup5Pose.getHeading())
                .build();*/

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup5Pose,  pickup6ControlPose, pickup6Pose))
                .setLinearHeadingInterpolation(pickup5Pose.getHeading(), pickup6Pose.getHeading())
                .build();
        landingPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup6Pose, landingPose)))
                .setLinearHeadingInterpolation(pickup6Pose.getHeading(), landingPose.getHeading())
                .build();*/
    }

    public void autonomousPathUpdate(){

        if (follower.isBusy()) return;

        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;
            case 1:
                // Path1 Shooter Path
                beginRTPauseForFinishedPath(1);
                setPathState(2);
                break;
            case 2:
                follower.followPath(grabPickup1Path1,true);
                setPathState(3);
                break;
            case 3:
                follower.followPath(grabPickup1Path2, true);
                setPathState(4);
                break;
            case 4:
                follower.followPath(scorePickup1, true);
                setPathState(5);
                break;
            case 5:
                // Path5 Shooter Path
                beginRTPauseForFinishedPath(4);
                setPathState(-3);
                break;
            /*case 5:
                if(!follower.isBusy() ) {
                    follower.followPath(grabPickup2,true);
                    follower.setMaxPower(1.0);
                    intakeArtifacts();
                    setPathState(-4);
                }
                break;
            case 6:
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
                if(!follower.isBusy() ) {
                    follower.followPath(grabPickup3,true);
                    follower.setMaxPower(1.0);
                    intakeArtifacts();
                    setPathState(-7);
                }
                break;
            case 9:
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
                follower.followPath(landingPath, true);
                follower.setMaxPower(1.0);
                setPathState(-11);
                break;*/
        }
    }
    // NEW: pre-spin during Path1 and Path4 (while they are running) but DO NOT feed
    // Path1 runs while pathState == 1, Path4 runs while pathState == 4
    private boolean isPreSpinState(int state) {
        return state == 1 || state == 4;
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
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
    private void setTurnPower(double turnCmd) {
        turnCmd = Range.clip(turnCmd, -MAX_TURN, MAX_TURN);

        double denom = Math.max(Math.abs(turnCmd), 1.0);

        frontLeftMotor.setPower((turnCmd) / denom);
        backLeftMotor.setPower((turnCmd) / denom);
        frontRightMotor.setPower((-turnCmd) / denom);
        backRightMotor.setPower((-turnCmd) / denom);
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
    private void updateRTPause() {
        if (!rtPauseActive) return;

        if (rtTimer.getElapsedTimeSeconds() >= RT_PAUSE_SECONDS) {
            endRTPause();
            return;
        }

        // Keep hood + flywheel commanded
        shootServo.setPosition(degToPos(activeShotHoodDeg));
        setFlywheelRPM(activeShotRPM);

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

        telemetry.addData("RT shotRPM", activeShotRPM);
        telemetry.addData("RT hoodDeg", activeShotHoodDeg);
        telemetry.addData("RT curTPS", curTPS);
        telemetry.addData("RT feedEnabled", feedEnabled);
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
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

        follower = Constants.createFollower(hardwareMap);
        // Drive motors for RT aiming pause
        frontLeftMotor  = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor   = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor  = hardwareMap.dcMotor.get("BRch0");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shootServo = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);

        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "Flywheelexp0");
        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "Flywheelexp2");
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardstopServo    = hardwareMap.servo.get("hardstopServo");
        hardstopServo.setPosition(0.55);
        /*limelight = hardwareMap.get(Limelight3A.class, "EIA Limelight");
        limelight.pipelineSwitch(0);
        limelight.start();*/
        buildPaths();
        follower.setStartingPose(startPose);
        hardstopServo.setPosition(0.55);
        //flywheelMotor.setVelocity(TARGET_TPS);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        //limelight.start();
        resetAimPid();
        rtPauseActive = false;
        feedEnabled = false;
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        endRTPause();
        rollerIntakeMotor.setPower(0);
        shootrollerServo.setPower(0);
        hardstopServo.setPosition(0.55);
        setTurnPower(0);
    }

}



