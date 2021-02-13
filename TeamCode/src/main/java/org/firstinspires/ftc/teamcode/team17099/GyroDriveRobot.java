package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

/**
 * Define a gyro drive using team bot
 */
public class GyroDriveRobot extends TeamRobot {
    public static final double HD_HEX_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor
    public static final double HD_UltraPlanetary_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor
    public static final double CORE_HEX_COUNTS_PER_ROTATION = 288; //  Rev Core Hex motor
    public static final double NERVEREST20_COUNTS_PER_ROTATION = 537.6;
    public static final double DRIVE_GEAR_REDUCTION = 0.77;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4;     // 4in Andymark HD Mecanum Wheels
    public static final double NERVEREST20_COUNTS_PER_INCH =
            (NERVEREST20_COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    //private static final double P_DRIVE_COEFF = 0.025;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.002;
    public static final double STRAFE_DISTANCE_MULTIPLIER = 0.7;

    // Tensor flow
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    public BNO055IMU imu;
    private LinearOpMode opMode;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    PIDController pidStrafe;


    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AWPSm1P/////AAABmfp26UJ0EUAui/y06avE/y84xKk68LTTAP3wBE75aIweAnuSt" +
                    "/zSSyaSoqeWdTFVB5eDsZZOP/N/ISBYhlSM4zrkb4q1YLVLce0aYvIrso" +
                    "GnQ4Iw/KT12StcpQsraoLewErwZwf3IZENT6aWUwODR7vnE4JhHU4+2Iy" +
                    "ftSR0meDfUO6DAb4VDVmXCYbxT//lPixaJK/rXiI4o8NQt59EIN/W0RqT" +
                    "ReAehAZ6FwBRGtZFyIkWNIWZiuAPXKvGI+YqqNdL7ufeGxITzc/iAuhJz" +
                    "NZOxGXfnW4sHGn6Tp+meZWHFwCYbkslYHvV5/Sii2hR5HGApDW0oDml6g" +
                    "OlDmy1Wmw6TwJTwzACYLKl43dLL35G";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    private VuforiaTrackables targetsUltimateGoal;
    private List<VuforiaTrackable> allTrackables;

    private TFObjectDetector tfod;

    public GyroDriveRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidStrafe = new PIDController(.01, .00003, 0);

        this.opMode = opMode;
        initGyroImu();
        initNavigation();
        initTfod();
    }

    private void initNavigation() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = super.webcam;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        phoneXRotate = 180;

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 9 * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 14.1f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 5.25f * mmPerInch;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        // WARNING:
        // In this sample, we do not wait for PLAY to be pressed.  Target Tracking is started immediately when INIT is pressed.
        // This sequence is used to enable the new remote DS Camera Preview feature to be used with this sample.
        // CONSEQUENTLY do not put any driving commands in this loop.
        // To restore the normal opmode structure, just un-comment the following line:

        targetsUltimateGoal.activate();
    }

    public void initGyroImu() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        // make sure the gyro is calibrated before continuing
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())  {
            opMode.telemetry.addData("Mode", "calibrating...");
            opMode.telemetry.update();
            opMode.sleep(50);
            opMode.idle();
        }

        composeTelemetry(opMode.telemetry);
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        this.tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        tfod.setZoom(1.5, 16.0/9.0);

        if (tfod != null) {
            tfod.activate();
        }
    }

    public void shutdownTfod() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public int getRingAmount(){
        if (tfod != null) {

            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                opMode.telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    opMode.telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    opMode.telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    opMode.telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    opMode.telemetry.update();

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

    public VuforiaTrackables getTargetsUltimateGoal() {
        return targetsUltimateGoal;
    }

    private void forward(double speed) {
        setPower(speed, speed, speed, speed);
    }

    /**
     * Turning robot by gyro. Left is positive
     *
     * @param speed
     * @param angle
     */
    public void gyroTurn(double speed, double angle) {
        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            opMode.telemetry.update();
        }
    }

    /**
     * Hold at the angle
     * @param speed
     * @param angle Left is positive
     * @param holdTime
     */
    public void gyroHold(double speed, double angle, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        stopAllMotors();
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        wheelFrontLeft.setPower(leftSpeed);
        wheelFrontRight.setPower(rightSpeed);
        wheelBackLeft.setPower(leftSpeed);
        wheelBackRight.setPower(rightSpeed);

        // Display it for the driver.
        opMode.telemetry.addData("Target", "%5.2f", angle);
        opMode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opMode.telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void gyroDrive(double speed, double distance, double angle) {
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            resetMotors();

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * NERVEREST20_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            int frontLeftTarget = wheelFrontLeft.getCurrentPosition()+ moveCounts;
            int frontRightTarget = wheelFrontRight.getCurrentPosition() + moveCounts;
            int backLeftTarget = wheelBackLeft.getCurrentPosition() + moveCounts;
            int backRightTarget = wheelBackRight.getCurrentPosition() + moveCounts;

            runToTarget(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            forward(speed);

            // keep looping while we are still active, and all motors are running.
            while (opMode.opModeIsActive() &&
                    wheelBackLeft.isBusy() &&
                    wheelBackRight.isBusy() &&
                    wheelFrontLeft.isBusy() &&
                    wheelFrontRight.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                setPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
                updateStrafeTelemetry(steer, frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            }

            resetMotors();
        }
    }

    private void updateStrafeTelemetry(double steer, int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        opMode.telemetry.addData("Steer",  "%5.2f", steer);
        opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",
                frontLeftTarget,  frontRightTarget, backLeftTarget, backRightTarget);
        opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",
                wheelFrontLeft.getCurrentPosition(),
                wheelFrontRight.getCurrentPosition(),
                wheelBackLeft.getCurrentPosition(),
                wheelBackRight.getCurrentPosition());
        opMode.telemetry.addData("Speed",   "%.2f:%.2f:%.2f:%.2f",
                wheelFrontLeft.getPower(), wheelFrontRight.getPower(),
                wheelBackLeft.getPower(), wheelBackRight.getPower());
        opMode.telemetry.update();
    }

    public void gyroStrafeSideway(double speed, double distance, double angle) {
        int moveCounts;
        double max;
        double error;
        double steer;
        double frontSpeed;
        double backSpeed;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            resetMotors();

            // Determine new target position, and pass to motor controller
            moveCounts = (int) Math.abs(distance * NERVEREST20_COUNTS_PER_INCH / STRAFE_DISTANCE_MULTIPLIER);

            // Set Target and Turn On RUN_TO_POSITION
            int sign = distance > 0 ? 1 : -1;
            int frontLeftTarget = wheelFrontLeft.getCurrentPosition() + sign * moveCounts;
            int frontRightTarget = wheelFrontRight.getCurrentPosition() - sign * moveCounts;
            int backLeftTarget = wheelBackLeft.getCurrentPosition() - sign * moveCounts;
            int backRightTarget = wheelBackRight.getCurrentPosition() + sign * moveCounts;


            // Set up parameters for driving in a straight line.
            pidStrafe.setSetpoint(0);
            pidStrafe.setOutputRange(0, speed);
            pidStrafe.setInputRange(-90, 90);
            pidStrafe.enable();

            runToTarget(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(sign * speed, -sign * speed, -sign * speed,
                    sign * speed);

            // keep looping while we are still active, and all motors are running.
            while (opMode.opModeIsActive() &&
                    wheelBackLeft.isBusy() &&
                    wheelBackRight.isBusy() &&
                    wheelFrontLeft.isBusy() &&
                    wheelFrontRight.isBusy()) {

                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
                Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

                steer = pidStrafe.performPID(angles.firstAngle);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                frontSpeed = speed - steer;
                backSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(frontSpeed), Math.abs(backSpeed));
                if (max > 1.0) {
                    frontSpeed /= max;
                    backSpeed /= max;
                }

                setPower(sign  * frontSpeed,
                        -sign * frontSpeed,
                        -sign  * backSpeed,
                        sign  *  backSpeed);

                // Display drive status for the driver.
                updateStrafeTelemetry(steer, frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            }

            // Turn off RUN_TO_POSITION
            resetMotors();
        }
    }

    private void runToTarget(int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        wheelFrontLeft.setTargetPosition(frontLeftTarget);
        wheelFrontRight.setTargetPosition(frontRightTarget);
        wheelBackLeft.setTargetPosition(backLeftTarget);
        wheelBackRight.setTargetPosition(backRightTarget);

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void resetMotors() {
        setPower(0, 0, 0, 0);
        wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        wheelFrontLeft.setPower(frontLeftSpeed);
        wheelFrontRight.setPower(frontRightSpeed);
        wheelBackLeft.setPower(backLeftSpeed);
        wheelBackRight.setPower(backRightSpeed);
    }

    private void stopAllMotors() {
        setPower(0, 0, 0, 0);
    }

    void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        /*
         // We don't use gravity
        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
         */
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public VectorF getLocation() {

        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                opMode.telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            opMode.telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            opMode.telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            opMode.telemetry.update();
            return translation;
        }
        else {
            opMode.telemetry.addData("Visible Target", "none");
            opMode.telemetry.update();
            return null;
        }
    }

    public void stopCamera() {
        // Disable Tracking when we are done;
        targetsUltimateGoal.deactivate();
    }
}