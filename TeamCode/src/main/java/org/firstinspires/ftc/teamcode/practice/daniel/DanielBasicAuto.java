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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team17099.TeamRobot;

import java.util.concurrent.TimeUnit;


/**
 *
 */

@Autonomous(name="DanielBasicAuto", group="Daniel's Autonomous")
public class DanielBasicAuto extends LinearOpMode {
    private AutoTeamBot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        //import the team bot so we have access to all the stuff in it.
        this.bot = new AutoTeamBot(hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //lift arm whole time so the wobble doesn't drop
        TimeUnit.MILLISECONDS.sleep(500);

        bot.closeGrabber();
        TimeUnit.MILLISECONDS.sleep(800);
        bot.liftArm();
        TimeUnit.MILLISECONDS.sleep(1000);

        //move the bot so it has space to turn
        bot.move(10,-1);
        TimeUnit.MILLISECONDS.sleep(500);

        //turn towards target zone
        bot.turn(8, -1);

        //move to target zone
        TimeUnit.MILLISECONDS.sleep(500);
        bot.move(45, -1);
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



        //move back from target zone
        bot.move(22, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn to left of field. Front facing left.
        bot.turn(20, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //move to launching area
        bot.move(30,-1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn right towards power shots
        bot.turn(5, 1);


        //fire 3 power shots by turning a little bit each time and having delay to let the function finish

        TimeUnit.MILLISECONDS.sleep(1000);
        bot.shoot(1.00);
        TimeUnit.MILLISECONDS.sleep(1500);

        //push ring to launcher
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.shoot(1.00);
        TimeUnit.MILLISECONDS.sleep(1000);
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.shoot(1.00);
        TimeUnit.MILLISECONDS.sleep(1000);
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);


        bot.stopShoot();
        TimeUnit.MILLISECONDS.sleep(1000);
    }
}
