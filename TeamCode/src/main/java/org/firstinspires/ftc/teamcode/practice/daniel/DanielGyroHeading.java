package org.firstinspires.ftc.teamcode.practice.daniel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;

@Autonomous(name="Daniel: Gyro Hold", group="Daniel's Teleops")
@Disabled
public class DanielGyroHeading extends LinearOpMode {
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

        bot.gyroHold(0.30, -45, 30);
//        bot.gyroHold(0.30, 45, 30);
    }
}