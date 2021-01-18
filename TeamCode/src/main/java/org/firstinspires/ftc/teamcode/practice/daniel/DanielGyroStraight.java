package org.firstinspires.ftc.teamcode.practice.daniel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;

@Autonomous(name="Daniel: Gryo Straight", group="Daniel's Teleops")
//@Disabled
public class DanielGyroStraight extends LinearOpMode {
    private GyroDriveRobot bot;
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        this.bot = new GyroDriveRobot(hardwareMap, this);

        telemetry.update();
        while (!isStarted()) {
            sleep(10);
            idle();
        }

        bot.gyroDrive(0.30, 50, 0);
//        bot.gyroDrive(0.30, -25, 0);
    }
}