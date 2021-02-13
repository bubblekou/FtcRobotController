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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.team17099.TeamRobot;
import java.util.concurrent.TimeUnit;
import java.util.List;



/**
 *
 */

@Autonomous(name="FinalAuto", group="Ethan's Autonomous")
public class FinalAuto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrsoGnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2IyftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqTReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJzNZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6gOlDmy1Wmw6TwJTwzACYLKl43dLL35G";

    private VuforiaLocalizer vuforia;

    public WebcamName WebcamName;


    private TFObjectDetector tfod;
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam 1");

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
    private AutonomousTeamRobot bot;

    public int ringAmount(){

        if (tfod != null) {

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
                    telemetry.update();

                    if(recognition.getLabel()==LABEL_FIRST_ELEMENT) {
                        return 4;
                    } else if (recognition.getLabel()==LABEL_SECOND_ELEMENT) {
                        return 1;
                    }

                }
            }
        }
        return 0;
    }
    private void TargetA() throws InterruptedException{
        //lift arm whole time so the wobble doesn't drop
        TimeUnit.MILLISECONDS.sleep(500);

        //move the bot so it has space to turn
        bot.move(20,-0.5);
        TimeUnit.MILLISECONDS.sleep(500);

        //turn towards target zone
        bot.turn(10, -1);

        //move to target zone
        TimeUnit.MILLISECONDS.sleep(500);
        bot.move(110, -0.5);
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
        bot.move(40, 0.5);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn towards high shot
        //usually scores the goal when the angle is around
        bot.turn(87, -1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //move to launching area
        bot.move(40,0.5);
        TimeUnit.MILLISECONDS.sleep(1000);

        bot.shoot(1);
        TimeUnit.MILLISECONDS.sleep(1500);

        //push ring to launcher
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1500);


        bot.stopShoot();
        TimeUnit.MILLISECONDS.sleep(1000);


        //move onto launch line
        bot.move(14,0.5);
        TimeUnit.MILLISECONDS.sleep(1000);
    }
    private void TargetB() throws InterruptedException{
        TimeUnit.MILLISECONDS.sleep(500);

        bot.turn(3, 1);

        //move the bot to target zone
        bot.move(190,-0.5);
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
        bot.move(36, 0.5);
        TimeUnit.MILLISECONDS.sleep(1000);

        //spin 180 degrees
        bot.turn(114, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn on flywheel
        bot.shoot(0.95);
        TimeUnit.MILLISECONDS.sleep(2000);

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
        bot.move(14,0.5);
        TimeUnit.MILLISECONDS.sleep(1000);
    }
    private void TargetC() throws InterruptedException{
        TimeUnit.MILLISECONDS.sleep(500);

        //move the bot so it has space to turn
        bot.move(20,-0.5);
        TimeUnit.MILLISECONDS.sleep(500);

        //turn towards target zone
        bot.turn(5, -1);

        //move to target zone
        TimeUnit.MILLISECONDS.sleep(500);
        bot.move(235, -0.5);
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
        bot.move(90, 0.5);
        TimeUnit.MILLISECONDS.sleep(1000);

        //turn towards high shot
        bot.turn(128, 1);
        TimeUnit.MILLISECONDS.sleep(1000);

        //move towards launch zone
        bot.move(14,0.5);
        TimeUnit.MILLISECONDS.sleep(1000);

        bot.shoot(1);
        TimeUnit.MILLISECONDS.sleep(1500);

        //push ring to launcher
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1000);
        bot.pushRing();
        TimeUnit.MILLISECONDS.sleep(1500);


        bot.stopShoot();
        TimeUnit.MILLISECONDS.sleep(1000);


        //move onto launch line
        bot.move(10,0.5);
        TimeUnit.MILLISECONDS.sleep(1000);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //import the team bot so we have access to all the stuff in it.
        this.bot = new AutonomousTeamRobot(hardwareMap);

        initVuforia();
        initTfod();
        tfod.setZoom(1.5, 16.0/9.0);

        if (tfod != null) {
            tfod.activate();
        }
        sleep (1000);
        double sum = 0;
        for (int i = 0; i < 10; i++){
            sum += ringAmount();
            sleep (100);
        }
        double x = Math.round(sum / 10.0);
        telemetry.addData("rings", ringAmount());
        telemetry.update();
        sleep (100);

        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        bot.closeGrabber();
        TimeUnit.MILLISECONDS.sleep(800);
        bot.liftArm();
        TimeUnit.MILLISECONDS.sleep(1000);

        if (x == 0){
            TargetA();
        }else if (x == 1){
            TargetB();
        }else{
            TargetC();
        }
    }
}
