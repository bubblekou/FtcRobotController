package org.firstinspires.ftc.teamcode.practice.daniel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;

@Autonomous(name="Daniel: Gyro Turn", group="Daniel's Teleops")
@Disabled
public class DanielGyroTurn extends LinearOpMode {
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

        bot.gyroTurn(0.30, 90);
//        bot.gyroTurn(0.30, -90);
    }
}