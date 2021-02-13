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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This file is Nah Robotic's autonomous teleop for ultimate goal.
 */
@Autonomous(name="17099: Auto", group="Team's Teleops")
public class AutoTeleop extends LinearOpMode {
    private GyroDriveRobot bot;

    @Override
    public void runOpMode() throws InterruptedException {
        //import the team bot so we have access to all the stuff in it.
        this.bot = new GyroDriveRobot(hardwareMap, this);
        while (!isStarted()) {
            sleep(100);
            idle();
        }

        double sum = 0;
        for (int i = 0; i < 10; i++){
            sum += bot.getRingAmount();
            sleep (100);
        }
        double rings = Math.round(sum / 10.0);
        telemetry.addData("rings", bot.getRingAmount());
        telemetry.update();
        bot.shutdownTfod();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (rings == 0){
            gotoTargetA();
        }else if (rings == 1){
            gotoTargetB();
        }else{
            gotoTargetC();
        }
    }

    private void gotoTargetA() throws InterruptedException{
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, -90);
        bot.gyroDrive(0.3, -20, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
    }

    private void gotoTargetB() throws InterruptedException{
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, 90);
        bot.gyroHold(0.30,90, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);
        bot.gyroDrive(0.3, -24, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
        bot.gyroDrive(0.3, 6, 0);
    }

    private void gotoTargetC() throws InterruptedException{
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, 90);
        bot.gyroHold(0.30,90, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);
        bot.gyroDrive(0.3, -40, 0);

        bot.gyroTurn(0.30, -90);
        bot.gyroHold(0.30,-90, 0.2);

        bot.gyroDrive(0.30, -24, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
        bot.gyroDrive(0.3, 24, 0);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);

        bot.gyroDrive(0.3, 16, 0);
    }
}
