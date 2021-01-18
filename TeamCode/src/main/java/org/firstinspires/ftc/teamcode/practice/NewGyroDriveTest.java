package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;
import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobotV2;

@Autonomous(name="Drive Avoid Imu", group="Exercises")
//@Disabled
public class NewGyroDriveTest extends LinearOpMode
{
    private GyroDriveRobotV2 bot;

    private double correction;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {
        this.bot = new GyroDriveRobotV2(hardwareMap, this);

        while (!isStarted()) {
            sleep(10);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // drive until end of period.
        bot.gyroDrive(0.2, 48, 0);
    }
}