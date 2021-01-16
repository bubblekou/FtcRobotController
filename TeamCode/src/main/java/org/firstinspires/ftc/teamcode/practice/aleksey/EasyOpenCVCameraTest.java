package org.firstinspires.ftc.teamcode.practice.aleksey;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class EasyOpenCVCameraTest extends LinearOpMode{
    OpenCvCamera webcam;
    //name of webcam - can be changed to fit
    final String camName = "Webcam 1";

    @Override
    public void runOpMode() {
        //instantiate openCvCamera, pass in display monitor
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, camName), cameraMonitorViewId);

        //specify the pipeline you want to use -> in this case the SamplePipeline
        webcam.setPipeline(new SamplePipeline());

        //open connection to webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //start streaming the view of the webcam
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        //add to and update telemetry
        telemetry.addLine("Waiting for start");
        telemetry.update();

        //wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            //send basic stats to telemetry
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            //the following will stop the camera streaming at the click of a button (in this case a)
            if (gamepad1.a) {
                webcam.stopStreaming();
                /*
                stopStreaming() is used to save processing power, to change where the stream goes
                use closeCameraDevice() instead
                 */
            }
            //save some processing power to avoid extra CPU cycles
            sleep(100);
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewPortPaused;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);

            return input;
        }

        @Override
        public void onViewportTapped() {
            viewPortPaused = !viewPortPaused;

            if (viewPortPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }



}
