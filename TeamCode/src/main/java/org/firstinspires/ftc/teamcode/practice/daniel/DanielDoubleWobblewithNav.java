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

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;

import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="DanielDoubleWobblewithNav", group="Daniel's Autonomous")
//@Disabled
public class DanielDoubleWobblewithNav extends LinearOpMode {
    private GyroDriveRobot bot;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        this.bot = new GyroDriveRobot(hardwareMap, this);

        while (!isStarted()) {
            sleep(100);
            idle();
        }

        while (!isStopRequested()) {
            VectorF lastLocation = bot.getLocation();
            if (lastLocation != null) {
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        lastLocation.get(0) / bot.mmPerInch, lastLocation.get(1) / bot.mmPerInch, lastLocation.get(2) / bot.mmPerInch);
            } else {
                telemetry.addData("Pos (in)", "unknown");
            }

            telemetry.update();
        }

        bot.liftArm();

        bot.gyroDrive(0.5, -60, 0);
        bot.gyroTurn(0.3, -90);
        bot.gyroHold(0.3, -90, 0.2);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
        bot.gyroTurn(0.3, 180);
        bot.gyroHold(0.3, 180, 0.2);
        bot.dropArm();

        VectorF lastLocation = bot.getLocation();
        long Xrobotord = (long) (lastLocation.get(0) / bot.mmPerInch);
        long Yrobotcord = (long) (lastLocation.get(1) / bot.mmPerInch);

        long wobblegoalleftposX = -48;
        long wobblegoalleftposY = -24;
        long angle = (long) ((long) 90 - tan(-1*(wobblegoalleftposY - Yrobotcord)/(wobblegoalleftposX - Xrobotord)));
        long distance = (long) sqrt((wobblegoalleftposX - Xrobotord)*(wobblegoalleftposX - Xrobotord) + (wobblegoalleftposY - Yrobotcord) * (wobblegoalleftposY - Yrobotcord));
        int drivedistance = (int) distance;

        bot.stopCamera();

        bot.gyroTurn(0.3, -angle);
        bot.gyroDrive(0.3, drivedistance - 6, 0);
        bot.flipGrabber();
        bot.liftArm();
        bot.gyroDrive(0.3, -(drivedistance + 6), 0);
        bot.gyroTurn(0.3, -90);
        bot.dropArm();
        bot.flipGrabber();
        bot.gyroDrive(0.3, 24, 0);
        bot.gyroTurn(0.3, 180);
        bot.gyroDrive(0.3, 12, 0);
    }
}