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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

@Autonomous(name = "TestEIARedWallAuto", group = "Decode2526")
@Disabled
public class TestEIARedWallAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pathState;
    //private final Pose startPose = new Pose(88.02, 6.95, Math.toRadians(-93)); // Start Pose of our robot.
    private final Pose startPose = new Pose(87.67, 8.193, Math.toRadians(-90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(88.28, 20.28, Math.toRadians(-115)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose pickup1Pose = new Pose(125.36, 35.64, Math.toRadians(90)); // Grab Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup1ControlPose = new Pose(78.66, 40.97, Math.toRadians(0));

    private final Pose pickup6Pose = new Pose(85.83, 81.12, Math.toRadians(-139)); // Score Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup6ControlPose = new Pose(81, 70, Math.toRadians(-139));
    private final Pose landingPose = new Pose(82.83, 69.28, Math.toRadians(-139)); // Landing Pose of our robot. It is facing the goal at a 135 degree angle.
    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, landingPath;


    // -------- Mechanisms --------
    private DcMotorEx flywheelMotor;      // velocity control
    private DcMotor rollerIntakeMotor;
    private CRServo shootrollerServo;   // feeder (CR)
    private Servo shootServo;         // hood (positional)

    // -------- Hood mapping + presets (tune these) --------
    private static final double MIN_POS = 0;
    private static final double MAX_POS = 1;
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;

    private double PRESET_HIGH_DEG = 40.0;

    // -------- Flywheel velocity control --------
    private static final double TICKS_PER_REV = 28.0;  // from your motor specs
    private static final double GEAR_RATIO    = 1.0;   // motor revs per flywheel rev

    // RPM targets
    private static final double TARGET_RPM    = 4500.0; // as requested

    // Feeding thresholds (hysteresis)
    private static final double RESUME_RPM_FRAC = 0.85; // resume feed at >= 85% of target
    private static final double PAUSE_RPM_FRAC  = 0.80; // pause feed if < 80% of target

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

    private double getBatteryVoltage(){
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor: hardwareMap.getAll(VoltageSensor.class)){
            double voltage = sensor.getVoltage();
            if (voltage >0) {
                min = Math.min(min, voltage);
            }
        }
        return (min == Double.POSITIVE_INFINITY) ? 0 : min;
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
    public void intakeArtifacts() {

        flywheelMotor.setVelocity(0.0);
        rollerIntakeMotor.setPower(INTAKE_POWER_PPG);
        shootrollerServo.setPower(FEED_REVERSE);

        if(pathTimer.getElapsedTimeSeconds() > 4 && pathState==-2){
            setPathState(20);//3
        }
        if(pathTimer.getElapsedTimeSeconds() >3 && pathState==-7){
            setPathState(20);//9
        }
    }
    public void enableShooter() {

        shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
        // Command target velocity
        flywheelMotor.setVelocity(TARGET_TPS);
        // Measure current speed
        double tps = Math.abs(flywheelMotor.getVelocity());
        telemetry.addData("tps ", tps);
        telemetry.addData("Flywheel RPM", TARGET_RPM);
        telemetry.addData("Resume/Pause RPM", TARGET_RPM*RESUME_RPM_FRAC);
        telemetry.addData("Feed Enabled", feedEnabled);
        telemetry.addData("Intake Power", "%.2f", rollerIntakeMotor.getPower());
        telemetry.addData("Feeder Pwr",  "%.2f", shootrollerServo.getPower());
        telemetry.addData("Hood Pos",    "%.3f", shootServo.getPosition());
        telemetry.update();
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
            shootrollerServo.setPower(FEED_FORWARD);
        } else {
            rollerIntakeMotor.setPower(0.0);
            shootrollerServo.setPower(0.0);
        }

        if(pathTimer.getElapsedTimeSeconds() > 12 && pathState ==-1) {
            setPathState(2);
            feedEnabled = false;
        }

    }
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,  pickup1ControlPose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /*scorePickup1 = follower.pathBuilder()
               .addPath(new BezierCurve(pickup1Pose,  pickupControlPose, pickup6Pose))
               .setLinearHeadingInterpolation(pickup5Pose.getHeading(), pickup6Pose.getHeading())
               .build();
        landingPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(pickup6Pose, landingPose)))
                .setLinearHeadingInterpolation(pickup6Pose.getHeading(), landingPose.getHeading())
                .build();*/
    }

    public void autonomousPathUpdate(){
        switch (pathState) {
            case 0:
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
                if(!follower.isBusy() ) {
                    follower.followPath(grabPickup1,true);
                    follower.setMaxPower(0.75);
                    intakeArtifacts();
                    setPathState(-2);
                }
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
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}



