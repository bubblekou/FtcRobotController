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

package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * This file is Nah Robotic's teleop. It is an advanced robot TeleOp with:
 * 1. mecanum wheel drivetrain
 * 2. an arm which can grab the wobble goal
 * 3. an intake system
 * 4. A launcher system
 *
 * This is includes 2 gamepads. It also has more functions that are used in TeamRobot that are essential to this.
 */

@TeleOp(name="17099: Manual", group="Team's Teleops")
public class ManualTeleop extends LinearOpMode {
    private TeamRobot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //import the team bot so we have access to all the stuff in it.
        this.bot = new TeamRobot(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Update turbo speed by dpad
            if (gamepad1.left_bumper) {
                bot.flipPace();
            }
            if (gamepad1.dpad_up) {
                bot.updateTurbo(true);
            }
            else if (gamepad1.dpad_down) {
                bot.updateTurbo(false);
            }
            //Strafe drive
            bot.strafe(gamepad1);

            //Intake
            if (gamepad1.dpad_right) {
                bot.inTake();
            }
            else if (gamepad1.dpad_left) {
                bot.outTake();
            }
            else {
                bot.stopTaking();
            }
            //stabilize the ring so it lies flat in the magazine
            if (gamepad1.right_bumper) {
                bot.stabilizeRing();
            }
            //launching system
            if (gamepad2.y) {
                bot.startFlywheel();
            }
            else {
                bot.stopFlywheel();
            }
            if (gamepad2.left_bumper) {
                bot.pushRing();
            }

            //wobble goal actions, including lifting it to put it over the wall and holding it in place
            if (gamepad2.dpad_up) {
                bot.liftArm();
            }
            else if (gamepad2.dpad_down) {
                bot.dropArm();
            }
            else {
                bot.stopArm();
            }
            if (gamepad2.right_bumper) {
                bot.flipGrabber();
            }
        }
    }
}
