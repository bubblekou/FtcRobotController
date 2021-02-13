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

package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.team17099.GyroDriveRobot;

import java.util.List;
import java.util.concurrent.TimeUnit;


/**
 *
 */

@Autonomous(name="AutoTeleop", group="Autonomous")
public class AutoTeleop extends LinearOpMode {
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
    private GyroDriveRobot bot;

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
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, -90);
        bot.gyroDrive(0.3, -20, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
    }
    private void TargetB() throws InterruptedException{
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, 90);
        bot.gyroHold(0.30,90, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);
        bot.gyroDrive(0.3, -24, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
        bot.gyroDrive(0.3, 6, 0);
    }
    private void TargetC() throws InterruptedException{
        bot.liftArm();

        //avoid the ring in the path
        bot.gyroDrive(0.3, -12, 0);
        bot.gyroStrafeSideway(0.3, 12, 0);
        bot.gyroDrive( 0.30, -42, 0);
        bot.gyroStrafeSideway(0.3, -12, 0);

        bot.gyroTurn(0.2, 90);
        bot.gyroHold(0.2, 90, 0.2);

        bot.gyroTurn(0.2, 180);
        bot.gyroHold(0.2, 180, 0.2);

        int count = 0;
        bot.startHighFlywheel();
        sleep(1000);

        while (opModeIsActive() && count < 4) {
            count++;
            telemetry.addData(">", "Ring " + count);
            telemetry.update();

            bot.pushRing();
            sleep(1000);
        }
        bot.gyroTurn(0.30, 90);
        bot.gyroHold(0.30,90, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);
        bot.gyroDrive(0.3, -40, 0);

        bot.gyroTurn(0.30, -90);
        bot.gyroHold(0.30,-90, 0.2);

        bot.gyroDrive(0.30, -24, 0);
        bot.dropArm();
        sleep(200);
        bot.flipGrabber();
        sleep(200);
        bot.liftArm();
        bot.gyroDrive(0.3, 24, 0);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);

        bot.gyroTurn(0.30, 0);
        bot.gyroHold(0.30,0, 0.2);

        bot.gyroDrive(0.3, 16, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        //import the team bot so we have access to all the stuff in it.
        this.bot = new GyroDriveRobot(hardwareMap, this);
        while (!isStarted()) {
            sleep(100);
            idle();
        }

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
        double rings = Math.round(sum / 10.0);
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

        sleep (100);

        if (rings == 0){
            TargetA();
        }else if (rings == 1){
            TargetB();
        }else{
            TargetC();
        }
    }
}
