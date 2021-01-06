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

package org.firstinspires.ftc.teamcode.practice.ethan;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;


/**
 *
 */

@Autonomous(name="EthanBasicAuto2", group="Ethan's Autonomous")
public class EthanBasicAuto2 extends LinearOpMode {
    private AutonomousTeamRobot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        //IMPORTANT: TURN THE ROBOT A LITTLE BIT LEFT BECAUSE THERE IS NO GYRO
        //import the team bot so we have access to all the stuff in it.
        this.bot = new AutonomousTeamRobot(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //lift arm whole time so the wobble doesn't drop
        TimeUnit.MILLISECONDS.sleep(500);

        bot.closeGrabber();
        TimeUnit.MILLISECONDS.sleep(800);
        bot.liftArm();
        TimeUnit.MILLISECONDS.sleep(1000);

        //move the bot to target zone
        bot.move(55,-1);
        TimeUnit.MILLISECONDS.sleep(1000);


        //drop off wobble
        bot.dropArm();
        TimeUnit.MILLISECONDS.sleep(500);
        bot.openGrabber();
        TimeUnit.MILLISECONDS.sleep(500);
        bot.liftArm();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.stopArm();
        TimeUnit.MILLISECONDS.sleep(1000);

        //move back from target zone and into launching area
        bot.move(18, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //spin 180 degrees
        bot.turn(41, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn on flywheel
        bot.shoot(1);
        TimeUnit.MILLISECONDS.sleep(1500);

        //push ring to launcher
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1500);

        //turn off flywheel
        bot.stopShoot();
        TimeUnit.MILLISECONDS.sleep(1000);


        //move onto launch line
        bot.move(7,1);
        TimeUnit.MILLISECONDS.sleep(1000);


    }
}
