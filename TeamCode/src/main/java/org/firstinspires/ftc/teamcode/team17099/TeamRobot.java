package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is our team robot, with all the functions necessary to run the Teleop in ManualTeleop.
 */

public class TeamRobot {
    private HardwareMap hardwareMap;
    // Mecanum dirvetrain. Grabber is in the front
    private DcMotor wheelFrontLeft = null;
    private DcMotor wheelFrontRight = null;
    private DcMotor wheelBackLeft = null;
    private DcMotor wheelBackRight = null;

    public double lx;
    public double rx;
    public double ly;

    public static final double HD_HEX_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor
    public static final double HD_UltraPlanetary_COUNTS_PER_ROTATION = 1120; //  Rev HD Hex motor
    public static final double CORE_HEX_COUNTS_PER_ROTATION = 288; //  Rev Core Hex motor

    public static final double DRIVE_GEAR_REDUCTION = 1.29;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 4;     // 4in Andymark HD Mecanum Wheels
    public static final double CORE_HEX_COUNTS_PER_INCH =
            (CORE_HEX_COUNTS_PER_ROTATION * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    private static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    private static final double P_DRIVE_COEFF = 0.075;     // Larger is more responsive, but also less stable

    // Macanum drive speed control with turbo between 0.1 and 1.0, there are face and slow paces
    // for turbo adjustment
    private double turbo = 0.5;
    private double pace = 0.2;

    // Intake system
    private DcMotor intake = null;
    private DcMotor conveyor = null;
    // Stablizer to help to put the ring in the magazine
    private Servo stabilizer = null;

    // Launching system
    private DcMotor flywheel = null;
    private Servo pusher = null;

    // Grabber system
    private DcMotor arm = null;
    private boolean isHeld = false;
    private Servo grabber = null;

    public BNO055IMU imu;

    public TeamRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheel_front_left");
        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheel_front_right");
        wheelBackLeft = hardwareMap.get(DcMotor.class, "wheel_back_left");
        wheelBackRight = hardwareMap.get(DcMotor.class, "wheel_back_right");

        wheelFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        wheelBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Initialize intake hardware
        conveyor = hardwareMap.get(DcMotor.class, "conveyor");
        conveyor.setDirection(DcMotor.Direction.FORWARD);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        stabilizer = hardwareMap.get(Servo.class, "stabilizer");

        //Initialize launching hardware
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        pusher = hardwareMap.get(Servo.class, "pusher");

        // Initialize wobble grabber hardware
        arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        grabber = hardwareMap.get(Servo.class, "grabber");
        this.isHeld = grabber.getPosition() == 0;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);
    }

    /**
     * Increases or decrease drive train turbo. Use higher turbo for fast speed and slower turbo
     * for better maneuver
     *
     * @param increasing
     */
    public void updateTurbo(boolean increasing) {
        if (increasing) {
            turbo = Math.min(0.75, turbo + pace);
        } else {
            turbo = Math.max(0.1, turbo - pace);
        }
    }

    /**
     * Tilerunner strafing, driving, and turning
     *
     * @param gamepad Gamepad
     */
    public void strafe(Gamepad gamepad) {
        lx = gamepad.left_stick_x;
        ly = gamepad.left_stick_y;
        rx = gamepad.right_stick_x;

        double wheelFrontRightPower = turbo * (-lx - rx - ly);
        double wheelBackRightPower = turbo * (lx - rx - ly);
        double wheelFrontLeftPower = turbo * (lx + rx - ly);
        double wheelBackLeftPower = turbo * (-lx + rx - ly);

        wheelFrontLeft.setPower(wheelFrontLeftPower);
        wheelFrontRight.setPower(wheelFrontRightPower);
        wheelBackLeft.setPower(0.9*wheelBackLeftPower);
        wheelBackRight.setPower(0.9*wheelBackRightPower);
    }

    public void strafetrig(Gamepad gamepad) {
        double r = Math.hypot(gamepad.left_stick_x, gamepad.left_stick_y);
        double robotAngle = Math.atan2(gamepad.left_stick_y, gamepad.left_stick_x) - Math.PI / 4;
        double rightX = gamepad.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        wheelFrontLeft.setPower(v1);
        wheelFrontRight.setPower(v2);
        wheelFrontRight.setPower(v3);
        wheelFrontLeft.setPower(v4);
    }
    public void move(int distance, int scale) throws InterruptedException{
        wheelFrontLeft.setPower(scale * 1.5);
        wheelFrontRight.setPower(scale *1.5);
        wheelBackLeft.setPower(scale *1.5);
        wheelBackRight.setPower(scale *1.5);

        TimeUnit.MILLISECONDS.sleep(10 * distance);

        wheelFrontLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelBackLeft.setPower(0);
        wheelBackRight.setPower(0);

    }
    //turns the robot
    //note: angle is not the actual angle it turns
    public void turn(int angle, int direction) throws InterruptedException{
        wheelFrontLeft.setPower(-1.5 * direction);
        wheelFrontRight.setPower(1.5  * direction);
        wheelBackLeft.setPower(-1.5  * direction);
        wheelBackRight.setPower(1.5  * direction);

        TimeUnit.MILLISECONDS.sleep(10 * angle);

        wheelFrontLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelBackLeft.setPower(0);
        wheelBackRight.setPower(0);

    }

    /**
     * Intake rings with pasta roller and pulley.
     */
    public void inTake() {
        intake.setPower(1.00);
        conveyor.setPower(5/6);
    }

    /**
     * Outtake rings in case ring is stuck or out of place
     */
    public void outTake() {
        intake.setPower(-1.00);
        conveyor.setPower(-5/6);
    }

    /**
     * Stop taking
     */
    public void stopTaking() {
        intake.setPower(0);
        conveyor.setPower(0);
    }

    /**
     * Flap the ring stabilizer to help to lay the ring in the magazine
     *
     * @throws InterruptedException
     */
    public void sweep() throws InterruptedException {
        stabilizer.setPosition(0);
        TimeUnit.MILLISECONDS.sleep(250);
        stabilizer.setPosition(1);
    }
    /*
    public void openStabilize() throws InterruptedException {
        stabilizer.setPosition(0);
    }
    public void closeStabilize() throws InterruptedException {
        stabilizer.setPosition(1);
    }
     */

    /**
     * pushing the ring into the launcher
     *
     * @throws InterruptedException
     */
    public void pushRing() throws InterruptedException {
        pusher.setPosition(1);
        TimeUnit.MILLISECONDS.sleep(300);
        pusher.setPosition(0);
    }

    /**
     * hold the wobble goal in place
     */
    public void flipGrabber() throws InterruptedException {
        if (isHeld) {
            grabber.setPosition(1);
        }
        else {
            grabber.setPosition(0);
        }
        isHeld = !isHeld;
        TimeUnit.MILLISECONDS.sleep(300);
    }

    public void gyroTurn(LinearOpMode opMode, double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
        }
    }

    public void gyroHold(LinearOpMode opMode, double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
        }

        // Stop all motion;
        wheelBackRight.setPower(0);
        wheelBackLeft.setPower(0);
        wheelFrontRight.setPower(0);
        wheelFrontLeft.setPower(0);
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

    public void gyroDrive(LinearOpMode opMode, double speed, double distance, double angle) {
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
            moveCounts = (int) (distance * CORE_HEX_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            int frontLeftTarget = wheelFrontLeft.getCurrentPosition()+ moveCounts;
            int frontRightTarget = wheelFrontRight.getCurrentPosition() + moveCounts;
            int backLeftTarget = wheelBackLeft.getCurrentPosition() + moveCounts;
            int backRightTarget = wheelBackRight.getCurrentPosition() + moveCounts;

            runToTarget(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            forward(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (isOffTarget(wheelFrontLeft, frontLeftTarget, 10)
                            && isOffTarget(wheelFrontRight, frontRightTarget, 10)
                            && isOffTarget(wheelBackLeft, backLeftTarget, 10)
                            && isOffTarget(wheelBackRight, backRightTarget, 10))) {

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
            }

            resetMotors();
        }
    }

    public void gyroStrafeSideway(LinearOpMode opMode, double speed, double distance, double angle) {
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
            moveCounts = (int) Math.abs(distance * CORE_HEX_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            int sign = distance > 0 ? 1 : -1;
            int frontLeftTarget = wheelFrontLeft.getCurrentPosition() + sign * moveCounts;
            int frontRightTarget = wheelFrontRight.getCurrentPosition() - sign * moveCounts;
            int backLeftTarget = wheelBackLeft.getCurrentPosition() - sign * moveCounts;
            int backRightTarget = wheelBackRight.getCurrentPosition() + sign * moveCounts;

            runToTarget(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(sign * speed, -sign * speed, -sign * speed,
                    sign * speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (isOffTarget(wheelFrontLeft, frontLeftTarget, 10)
                            && isOffTarget(wheelFrontRight, frontRightTarget, 10)
                            && isOffTarget(wheelBackLeft, backLeftTarget, 10)
                            && isOffTarget(wheelBackRight, backRightTarget, 10))) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

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

                setPower(sign * frontSpeed, -sign * frontSpeed, -sign * backSpeed, sign * backSpeed);

                // Display drive status for the driver.
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

    private boolean isOffTarget(DcMotor motor, int target, int tolerance) {
        return Math.abs(motor.getCurrentPosition() - target) >= tolerance;
    }

    private void forward(double speed) {
        setPower(speed, speed, speed, speed);
    }

    public void setPower(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        wheelFrontLeft.setPower(frontLeftSpeed);
        wheelFrontRight.setPower(frontRightSpeed);
        wheelBackLeft.setPower(backLeftSpeed);
        wheelBackRight.setPower(backRightSpeed);
    }

    private void stop() {
        setPower(0, 0, 0, 0);
    }

    /**
     * lifting the arm that will hold the wobble goal so that it can clear the perimeter
     */
    public void liftArm() {
        arm.setPower(0.3);
    }
    public void dropArm() {
        arm.setPower(-0.3);
    }
    public void stopArm() {
        arm.setPower(0);
    }
    /**
     * flywheel for launcher
     */
    public void startLowFlywheel() {
        flywheel.setPower(0.7);
    }
    public void startHighFlywheel() {
        flywheel.setPower(1);
    }
    public void stopFlywheel() {
        flywheel.setPower(0);
    }
    public void shootPowerShot() { flywheel.setPower(0.75);}

    public void shootFarRing() {
        flywheel.setPower(0.8);
    }
}
