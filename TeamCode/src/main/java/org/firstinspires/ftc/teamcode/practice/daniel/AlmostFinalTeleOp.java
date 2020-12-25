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

package org.firstinspires.ftc.teamcode.practice.daniel;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AlmostFinalTeleOp", group="Linear Opmode")
//@Disabled
public class AlmostFinalTeleOp extends LinearOpMode {

    public DcMotor conveyor = null;
    public DcMotor intake = null;
    public DcMotor flywheel = null;
    public DcMotor grabber = null;
    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelFrontRight = null;
    public DcMotor wheelBackLeft = null;
    public DcMotor wheelBackRight = null;

    public Servo wobble_goal_grabber = null;
    public Servo pusher = null;
    public Servo stabilizer = null;

    public double turbo = 0.5;

    private int nextwobble_goal_grabber = 0;
    private int nextStabilizer = 0;
    private int nextPusher = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheel_front_left");
        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheel_front_right");
        wheelBackLeft = hardwareMap.get(DcMotor.class, "wheel_back_left");
        wheelBackRight = hardwareMap.get(DcMotor.class, "wheel_back_right");

        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        grabber = hardwareMap.get(DcMotor.class, "grabber");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        wobble_goal_grabber = hardwareMap.get(Servo.class, "wobble_goal_grabber");
        pusher = hardwareMap.get(Servo.class, "pusher");
        stabilizer = hardwareMap.get(Servo.class, "stabilizer");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        conveyor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(DcMotorSimple.Direction.FORWARD);

        wheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double wheelFrontRightPower = turbo * (-lx - rx - ly);
            double wheelBackRightPower = turbo * (lx - rx - ly);
            double wheelFrontLeftPower = turbo * (lx + rx - ly);
            double wheelBackLeftPower = turbo * (-lx + rx - ly);

            double grabberpower;

            if (gamepad2.dpad_down) {
                grabberpower = 1;
            }
            else if (gamepad2.dpad_up) {
                grabberpower = -1;
            }
            else {
                grabberpower = 0;
            }

            double intakePower = 0.00;
            double conveyorPower = 0.00;
            double flywheelpower = 0.00;

            if (gamepad2.left_bumper) {
                nextPusher++;
                if(nextPusher % 2 == 0){
                    pusher.setPosition(1);
                } else {
                    pusher.setPosition(0);
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }
            if (gamepad2.a) {
                nextwobble_goal_grabber++;
                if(nextwobble_goal_grabber % 2 == 0){
                    wobble_goal_grabber.setPosition(1);
                } else {
                    wobble_goal_grabber.setPosition(0);
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }
            if (gamepad2.right_bumper) {
                nextStabilizer++;
                if(nextStabilizer % 2 == 0){
                    stabilizer.setPosition(1);
                } else {
                    stabilizer.setPosition(0);
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (gamepad1.dpad_up) {
                intakePower = 1.00;
                conveyorPower = 1.00;
            }
            else if (gamepad1.dpad_down) {
                intakePower = -1.00;
                conveyorPower = -1.00;
            }
            else {
                intakePower = 0.00;
                conveyorPower = 0.00;
            }
            if (gamepad2.x) {
                flywheelpower = 1.00;
            }
            else if (gamepad2.y) {
                flywheelpower = -1.00;
            }
            else {
                flywheelpower = 0.00;
            }
            wheelFrontLeft.setPower(wheelFrontLeftPower);
            wheelFrontRight.setPower(wheelFrontRightPower);
            wheelBackLeft.setPower(wheelBackLeftPower);
            wheelBackRight.setPower(wheelBackRightPower);

            intake.setPower(intakePower);
            conveyor.setPower(conveyorPower);
            flywheel.setPower(flywheelpower);
            grabber.setPower(grabberpower);
        }
    }
}
