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

package org.firstinspires.ftc.teamcode.Archive; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "EIARedWallAutoV3", group = "Decode2526")

public class EIARedWallAutoV3 extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pathState;
    private boolean rtPauseActive = false;
    private Timer rtTimer = new Timer();
    private boolean ltActive = false;
    private static final double MAX_TURN   = 0.6;
    private final Pose startPose = new Pose(79, 7.625,Math.toRadians(270)); // Start Pose of our robot.
    private final Pose Path1 = new Pose(89, 17, Math.toRadians(250)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose Path2 = new Pose(94, 60);
    private final Pose Path3 = new Pose(131, 60);
    private final Pose Path4 = new Pose(131, 70);
    private final Pose Path4ControlPose = new Pose(90, 71);
    private final Pose Path5 = new Pose(84, 84);
    private final Pose Path5ControlPose = new Pose(81, 71);
    private final Pose Path6 = new Pose(85, 84);
    private final Pose Path7 = new Pose(131, 84);
    private final Pose Path8 = new Pose(84, 84);
    private final Pose Spike1Sprint = new Pose(94, 35.44);
    private final Pose Spike1Intake = new Pose(131, 35.5);
    private final Pose Spike1Shoot = new Pose(89, 17);
    private final Pose Path9 = new Pose(115, 70);

    private PathChain scorePreload,grabPickup1Path1,grabPickup1Path2, scorePickup2,releaseGatePath,rotateinPlacePath,grabPickup2Path1,scorePickup3,grabPickup3Path1,grabPickup3Path2,scorePickup4,landingpath;
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
    private static final double X_OFFSET = +14;
    private static final double Y_OFFSET = -8;
    // RPM targets
    private static final double TARGET_RPM    = 4500.0;//4500;//4500.0; // as requested
    private static final double MAX_RPM       = 4500.0;

    private static final double SHOT_A_RPM = 4500.0;
    private static final double SHOT_A_HOOD_DEG = 33.0;

    // After Path 8, 11, 14 -> 4100 RPM, 27 deg
    private static final double SHOT_B_RPM = 3400.0;
    private static final double SHOT_B_HOOD_DEG = 36.0;

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
    private static final double RT_PAUSE_SECONDS_FAR = 3;
    private static final double RT_PAUSE_SECONDS_CLOSE = 1.5;

    // -------- Intake/feeder powers --------
    private static final double INTAKE_POWER = 1.0;
    private static final double FEED_FORWARD = -1.0; // forward
    private static final double FEED_REVERSE = +1.0; // reverse

    // State: feeding allowed while RT is held
    private boolean feedEnabled = false;

    private static Pose offXY(Pose s) {
        if (Math.abs(s.getX() - 89.0) < 1e-9 && Math.abs(s.getY() - 17.0) < 1e-9) {
            return s;
        } else if (Math.abs(s.getX() - 84.0) < 1e-9 && Math.abs(s.getY() - 84.0) < 1e-9) {
            return new Pose(s.getX() + 6, s.getY()); // your special-case rule
        } else {
            return new Pose(s.getX() - X_OFFSET, s.getY() + Y_OFFSET);
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
        return state == 4 || state == 5   // Path3 running
                || state == 9 || state == 10   // Path9 running
                || state == 13 || state == 14;  // Path12 running
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
        if (finishedPathIndex == 1 || finishedPathIndex == 14) {
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
                .addPath(new BezierLine(offXY(startPose), offXY(Path1)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(245))
                .build();

        grabPickup1Path1 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path1), offXY(Path2)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();

        grabPickup1Path2 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path2), offXY(Path3)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        releaseGatePath = follower.pathBuilder()
                .addPath(new BezierCurve(offXY(Path3), offXY(Path4ControlPose), offXY(Path4)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(offXY(Path4), offXY(Path5ControlPose), offXY(Path5)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        rotateinPlacePath = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path5), offXY(Path6)))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        grabPickup2Path1 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path6), offXY(Path7)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path7), offXY(Path8)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(225))
                .build();

        grabPickup3Path1 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Path8), offXY(Spike1Sprint)))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(0))
                .build();

        grabPickup3Path2 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Spike1Sprint), offXY(Spike1Intake)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Spike1Intake), offXY(Spike1Shoot)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(245))
                .build();

        landingpath = follower.pathBuilder()
                .addPath(new BezierLine(offXY(Spike1Shoot), offXY(Path9)))
                .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(0))
                .build();
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
                follower.setMaxPower(0.25);
                follower.followPath(grabPickup1Path2, true);
                follower.setMaxPower(1);
                setPathState(4);
                break;
            case 4:
                follower.followPath(releaseGatePath, true);
                setPathState(5);
                break;
            case 5:
                follower.followPath(scorePickup2, true);
                setPathState(6);
                break;
            case 6:
                beginRTPauseForFinishedPath(6);
                setPathState(7);
                break;
            case 7:
                follower.followPath(rotateinPlacePath, true);
                setPathState(8);
                break;
            case 8:
                follower.setMaxPower(0.25);
                follower.followPath(grabPickup2Path1, true);
                follower.setMaxPower(1);
                setPathState(9);
                break;
            case 9:
                follower.followPath(scorePickup3, true);
                setPathState(10);
                break;
            case 10:
                beginRTPauseForFinishedPath(10);
                setPathState(11);
                break;
            case 11:
                follower.followPath(grabPickup3Path1,true);
                setPathState(12);
                break;
            case 12:
                follower.setMaxPower(0.25);
                follower.followPath(grabPickup3Path2, true);
                follower.setMaxPower(1);
                setPathState(13);
                break;
            case 13:
                follower.followPath(scorePickup4, true);
                setPathState(14);
                break;
            case 14:
                beginRTPauseForFinishedPath(14);
                setPathState(15);
                break;
            case 15:
                follower.followPath(landingpath,true);
                setPathState(-1);
                break;
        }
    }
    // NEW: pre-spin during Path1 and Path4 (while they are running) but DO NOT feed
    // Path1 runs while pathState == 1, Path4 runs while pathState == 4
    private boolean isPreSpinState(int state) {
        return state == 1 || state == 6 || state == 12;
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

        if (pathState == 6 || pathState == 10){
            if (rtTimer.getElapsedTimeSeconds() >= RT_PAUSE_SECONDS_CLOSE) {
                endRTPause();
                return;
            }
        } else {
            if (rtTimer.getElapsedTimeSeconds() >= RT_PAUSE_SECONDS_FAR) {
                endRTPause();
                return;
            }
        }

        if (rtTimer.getElapsedTimeSeconds() >= RT_PAUSE_SECONDS_FAR) {
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

        buildPaths();
        follower.setStartingPose(startPose);
        hardstopServo.setPosition(0.55);

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



