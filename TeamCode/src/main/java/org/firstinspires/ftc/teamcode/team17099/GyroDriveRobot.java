package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Define a gyro drive using team bot
 */
public class GyroDriveRobot extends TeamRobot {
    private LinearOpMode opMode;

    public GyroDriveRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);
        init();

        calibrateGyro(opMode);
        this.opMode = opMode;
    }

    public void calibrateGyro(LinearOpMode linearOpMode) {
        // make sure the gyro is calibrated before continuing
        while (!linearOpMode.isStopRequested() && !imu.isGyroCalibrated())  {
            linearOpMode.sleep(50);
            linearOpMode.idle();
        }
    }


}
