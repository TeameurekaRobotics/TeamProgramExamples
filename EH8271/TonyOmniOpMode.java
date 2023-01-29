/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.ExampleCode.EH8271;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TonyOmniOpMode", group="Tony")
//@Disabled
public class TonyOmniOpMode extends LinearOpMode {

    TonyRobotHardware robot       = new TonyRobotHardware(this);

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        boolean isUp = true;

        robot.init();
        robot.ResetDriveEncoders();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();
        runtime.reset();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            double maxSpeed = 0.3 + gamepad1.right_trigger;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.left_stick_y;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = (axial - lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial + lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
           /*
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 0.6) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
*/
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            leftFrontPower = Range.clip(leftFrontPower, -maxSpeed, maxSpeed);
            leftBackPower = Range.clip(leftBackPower, -maxSpeed, maxSpeed);
            rightFrontPower = Range.clip(rightFrontPower, -maxSpeed, maxSpeed);
            rightBackPower = Range.clip(rightBackPower, -maxSpeed, maxSpeed);


            // Send calculated power to wheels
            robot.frontLeft.setPower(leftFrontPower);
            robot.frontRight.setPower(rightFrontPower);
            robot.backLeft.setPower(leftBackPower);
            robot.backRight.setPower(rightBackPower);

            //update odometry
            robot.CalculateOdometry();

            //region HAND CONTROLS

            /*spin fingers

                robot.fingerL.setPosition(-gamepad2.right_stick_y + 0.5);
                robot.fingerR.setPosition(gamepad2.right_stick_y + 0.5);
             */

            //open palm (x)
            if (gamepad2.b)
            {
                //open palm
                robot.palmL.setPosition(0.5); // center
                robot.palmR.setPosition(0.5); // center
            }

            //close palm (o)
            if (gamepad2.a)
            {
                robot.palmL.setPosition(1); // right limit
                robot.palmR.setPosition(0); // left limit
            }

            //swing arm
            if(gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1)
            {
                robot.armMotor.setPower(gamepad2.right_stick_y/0.5);
                robot.armHoldpos = robot.armMotor.getCurrentPosition();
            }

            else
            {
                robot.armMotor.setPower((double)(robot.armMotor.getCurrentPosition() - robot.armHoldpos) / robot.slopeVal);
            }

            //spin wrist
            if(gamepad2.right_stick_x > 0.5 && isUp)
            {
                robot.wrist.setPosition(0);
                isUp = false;
            }

            if(gamepad2.right_stick_x < -0.5 && !isUp)
            {
                robot.wrist.setPosition(1);
                isUp = true;
            }

            telemetry.addData("Is Up", isUp);

            //endregion

            //lift

            robot.liftL.setPower((gamepad2.right_trigger) - (gamepad2.left_trigger));
            robot.liftR.setPower((-gamepad2.right_trigger) + (gamepad2.left_trigger));

            if(gamepad2.y)
            {
                RunLiftAuto();
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            //arm pos and hold
            telemetry.addData("arm pos", "%6d", robot.armMotor.getCurrentPosition());
            telemetry.addData("arm hold pos", robot.armHoldpos);

            telemetry.addData("lift pos: L,R","%6d  %6d", robot.liftL.getCurrentPosition(), robot.liftR.getCurrentPosition());

            //Show Odometry and encoder values
            telemetry.addData("LRA", "%6d    %6d    %6d", robot.currentLeftPos, robot.currentRightPos, robot.currentAuxPos);
            telemetry.addData("xyh", "%6.1f cm     %6.1f cm     %6.1f deg", robot.pos.x, robot.pos.y, Math.toDegrees(robot.pos.h));
            telemetry.addData("loop", "%.1f ms", timer.milliseconds());
            timer.reset();
            telemetry.update();
        }

    }

    public void RunLiftAuto()
    {
        //set target position for lift
        robot.liftL.setTargetPosition(robot.liftLTarPos);
        robot.liftL.setPower(0.5);
        robot.liftL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.liftR.setTargetPosition(robot.liftRTarPos);
        robot.liftR.setPower(0.5);
        robot.liftR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set target position for swing arm
        robot.armMotor.setTargetPosition(robot.armTarPos);
        robot.armMotor.setPower(0.5);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(robot.liftL.isBusy())
        {
            telemetry.addData("driving to...", "%7d :%7d", robot.liftLTarPos, robot.liftRTarPos);
            telemetry.addData("currently at...", "%7d :%7d",
                    robot.liftL.getCurrentPosition(),
                    robot.liftR.getCurrentPosition());

            telemetry.addData("arm driving to...", robot.armTarPos);
            telemetry.addData("currently at...", robot.armMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}

