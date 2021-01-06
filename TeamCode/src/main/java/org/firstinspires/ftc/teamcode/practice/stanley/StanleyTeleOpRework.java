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

package org.firstinspires.ftc.teamcode.practice.stanley;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.team17099.TeamRobot;

import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Stanley: AlmostFinalTeleOp", group="Stanley's Teleops")
@Disabled

public class StanleyTeleOpRework extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private TeamRobot bot;

    public Servo wobble_goal_grabber = null;
    public Servo pusher = null;

    public DcMotor flywheel = null;
    public DcMotor grabber = null;

    public double turbo = 0.5;

    private int nextwobble_goal_grabber = 0;
    private int nextStabilizer = 0;
    private int nextPusher = 0;
    private int startingRings=0;
    public static final String VUFORIA_KEY = "AXIXrHj/////AAABmeEMqruXWUCBuoatjfPPvO" +
            "Qv4U/tRYoBqMMvXyAoHLHWYYYQSPx3ZOZ7GcdOCuTHK5HYM6oJ4gX1ZTxVec9RI4xa5ZOSPgTQvSo" +
            "Er2GJeRPohMXHEy6DRer3JDhvcPN32CzBiKJf2i60dFivASvEyU2EGRHGKq41VjsOk09o2q0Wr9ly" +
            "oEzdhNjMgAf8OfPn8wl93IM0Bo2+hH0ZtUSmZUoyBu53qlB0wgZ+FJYHxOOXdhim0ka+qa0CkFOkn" +
            "lN35bbLE6yNSyBOV86FaSZ0UuBNXfCX4O0IWh7qSBXcU/cQVMw3faOu8Hx3LiReY1lcQ1I4q0QP05" +
            "IUr5l71eQEMFLO71ByBWG95IkHucF5iyrA";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        this.bot = new TeamRobot(hardwareMap);

        bot.init();

        grabber = hardwareMap.get(DcMotor.class, "grabber");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");

        wobble_goal_grabber = hardwareMap.get(Servo.class, "wobble_goal_grabber");
        pusher = hardwareMap.get(Servo.class, "pusher");


        flywheel.setDirection(DcMotor.Direction.REVERSE);
        grabber.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData("rings",startingRings);
                        if(recognition.getLabel()== "LABEL_FIRST_ELEMENT") {
                            startingRings = 1;
                        }else{
                            startingRings = 4;
                        }
                    }
                    telemetry.update();
                }

            }

            if (tfod != null) {
                tfod.shutdown();
            }
            //Update turbo speed by dpad
            if (gamepad1.dpad_up) {
                bot.updateTurbo(true);
            }
            else if (gamepad1.dpad_down) {
                bot.updateTurbo(false);
            }
            //Strafe drive
            bot.strafe(gamepad1);

            double grabberpower;

            if (gamepad2.dpad_down) {
                grabberpower = 1;
            }
            else if (gamepad2.dpad_up) {
                grabberpower = -1;
            }
            else {
                grabberpower = 0;
            }

            double flywheelpower = 0.00;

            if (gamepad2.left_bumper) {
                nextPusher++;
                if(nextPusher % 2 == 0){
                    pusher.setPosition(1);
                } else {
                    pusher.setPosition(0);
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }
            if (gamepad2.right_bumper) {
                nextwobble_goal_grabber++;
                if(nextwobble_goal_grabber % 2 == 0){
                    wobble_goal_grabber.setPosition(1);
                } else {
                    wobble_goal_grabber.setPosition(0);
                }
                TimeUnit.MILLISECONDS.sleep(500);
            }
            if (gamepad1.right_bumper) {
                bot.stabilizeRing();
            }

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


            if (gamepad2.y) {
                flywheelpower = 1.00;
            }
            else if (gamepad2.x) {
                flywheelpower = -1.00;
            }
            else {
                flywheelpower = 0.00;
            }

            flywheel.setPower(flywheelpower);
            grabber.setPower(grabberpower);
        }
    }
}
