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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="RobotCentricOpMode_Linear", group="Linear OpMode")
@Disabled
public class RobotCentricOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor frontLeftMotor = null, backLeftMotor = null, frontRightMotor = null, backRightMotor = null;
    private DcMotor flywheelMotor = null, rollerIntakeMotor = null;
    CRServo shootrollerServo = null;
    Servo shootServo = null;
    private double SLIDE_POWER = 0.5;
    private double CLIMB_POWER = 0.7;
    int ARM_JUNCTION_LENGTH = 10;

    private static final double GEAR_RATIO = 1.3;


    private static final double WRIST_FORWARD_POSITION = 0.6; // 180 degrees
    private static final double WRIST_BACKWARD_POSITION = 0.0; // 0 degrees
    private boolean isWristForward = false; // Tracks the current position of the vertical wrist
    private boolean dpadPressed = false;   // Prevents rapid toggling for vertical wrist

    private static final double oWRIST_FORWARD_POSITION = 0.5; // 90 degrees
    private static final double oWRIST_BACKWARD_POSITION = 0.0; // 0 degrees
    private boolean oisWristForward = false; // Tracks the current position of the vertical wrist
    private boolean odpadPressed = false;   // Prevents rapid toggling for vertical wrist

    private static final double hWRIST_FORWARD_POSITION = 0.4; // 180 degrees
    private static final double hWRIST_BACKWARD_POSITION = 0.1; // 0 degrees
    private boolean ishWristForward = false; // Tracks the current position of the horiz wrist
    private boolean hdpadPressed = false;   // Prevents rapid toggling for vertical wrist

    private static final double MIN_POS = 0;  // example: your measured bottom
    private static final double MAX_POS = 1;  // example: your measured top

    // If you measured hood angles, set these (example values):
    private static final double HOOD_MIN_DEG = 0.0;
    private static final double HOOD_MAX_DEG = 40.0;

    // Presets in DEGREES (edit to taste)
    private static final double PRESET_LOW_DEG  = 10.0;
    private static final double PRESET_MID_DEG  = 24.0;
    private static final double PRESET_HIGH_DEG = 35.0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Declare our motors
        // Make sure your ID's match your configuration
        frontLeftMotor = hardwareMap.dcMotor.get("Flch3");
        backLeftMotor = hardwareMap.dcMotor.get("Blch2");
        frontRightMotor = hardwareMap.dcMotor.get("FRch1");
        backRightMotor = hardwareMap.dcMotor.get("BRch0");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shootServo = hardwareMap.servo.get("shootexpservo1");
        shootServo.setDirection(Servo.Direction.REVERSE);
        shootrollerServo = hardwareMap.crservo.get("shootrollexpservo2");

        flywheelMotor = hardwareMap.dcMotor.get("Flywheelexp0");
        rollerIntakeMotor = hardwareMap.dcMotor.get("Rollerintakeexp1");

        double hoodStart = 0.5 * (MIN_POS + MAX_POS);
        shootServo.setPosition(hoodStart);



       // vSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // vSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested())
        {
            return;
        }

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is not reversed
            double x = gamepad1.left_stick_x * 1.3; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            if(gamepad1.right_trigger !=0){
                telemetry.addData("RB: intake position", rollerIntakeMotor.getCurrentPosition() );
                rollerIntakeMotor.setTargetPosition(2000); //4475 //4575
                rollerIntakeMotor.setPower(1.0);
                rollerIntakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                rollerIntakeMotor.setPower(0);
            }
            if(gamepad1.left_trigger !=0) {
                telemetry.addData("RB: intake position", flywheelMotor.getCurrentPosition());
                flywheelMotor.setTargetPosition(2000); //4475 //4575
                flywheelMotor.setPower(1.0);
                flywheelMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                flywheelMotor.setPower(0);
            }
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                shootrollerServo.setPower(1.0);  // spin continuously forward
            }else if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                shootrollerServo.setPower(-1.0);  // spin continuously forward
            } else {
                shootrollerServo.setPower(0.0);  // stop spinning
            }

            if (gamepad1.dpad_down) {
                shootServo.setPosition(degToPos(PRESET_LOW_DEG));
            } else if (gamepad1.dpad_left) {
                shootServo.setPosition(degToPos(PRESET_MID_DEG));
            } else if (gamepad1.dpad_up) {
                shootServo.setPosition(degToPos(PRESET_HIGH_DEG));
            }


            // --- Telemetry (for debugging/feedback) ---
            telemetry.addData("Right Bumper", gamepad1.right_bumper);
            telemetry.addData("Servo Power", gamepad1.right_bumper ? 1.0 : 0.0);
            telemetry.update();

// --- CR Servo #2 (shootServo) ---
            //double shootPower = gamepad1.left_bumper ? -1.0 : 0.0;  // spin opposite when LB held
            //shootServo.setPower(shootPower);

            telemetry.addData("LB pressed", gamepad1.left_bumper);
            //telemetry.addData("Shoot Servo power", shootPower);

            telemetry.update();


        }

    }
    private double degToPos(double deg){
        double pos = Range.scale(deg, HOOD_MIN_DEG, HOOD_MAX_DEG, MIN_POS, MAX_POS);
        return Range.clip(pos, MIN_POS, MAX_POS);
    }


}



