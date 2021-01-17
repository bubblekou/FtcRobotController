package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
    private static final double P_DRIVE_COEFF = 0.075;     // Larger is more responsive, but also less stable

    public BNO055IMU imu;
    private LinearOpMode opMode;

    public GyroDriveRobot(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);
        init();

        wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        this.opMode = opMode;
    }

    public void calibrateGyro() {
        // make sure the gyro is calibrated before continuing
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())  {
            opMode.sleep(50);
            opMode.idle();
        }
    }

    private void forward(double speed) {
        setPower(speed, speed, speed, speed);
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
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
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
            moveCounts = (int) Math.abs(distance * NERVEREST20_COUNTS_PER_INCH);

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

                setPower(sign * frontSpeed, -sign * frontSpeed, -sign * 0.6 * backSpeed, sign * 0.6 * backSpeed);

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

    public void setPower(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        wheelFrontLeft.setPower(frontLeftSpeed);
        wheelFrontRight.setPower(frontRightSpeed);
        wheelBackLeft.setPower(backLeftSpeed);
        wheelBackRight.setPower(backRightSpeed);
    }

    private void stop() {
        setPower(0, 0, 0, 0);
    }
}
