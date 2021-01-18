package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

public class GyroDriveRobotV2 extends TeamRobot {
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;
    public BNO055IMU imu;

    private LinearOpMode opMode;

    // State used for updating telemetry
    private Orientation angles;
    private Acceleration gravity;

    public GyroDriveRobotV2(HardwareMap hardwareMap, LinearOpMode opMode) {
        super(hardwareMap);
        init();

        wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu2");
        imu.initialize(parameters);

        this.opMode = opMode;
        calibrateGyro();
    }

    public void calibrateGyro() {
        // make sure the gyro is calibrated before continuing
        while (!opMode.isStopRequested() && !imu.isGyroCalibrated()) {
            opMode.sleep(50);
            opMode.idle();
        }

        composeTelemetry(opMode.telemetry);
    }

    void composeTelemetry(Telemetry telemetry) {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
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

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        if (degrees == 0) {
            return;
        }

        double leftPower, rightPower;
        if (degrees < 0) {   // turn right.
            leftPower = power;
            rightPower = -power;
        } else {   // turn left.
            leftPower = -power;
            rightPower = power;
        }

        // set power to rotate.
        setPower(leftPower, rightPower, leftPower, rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0) {
            }

            while (opMode.opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        stopAllMotors();

        // wait for rotation to stop.
        opMode.sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void setPower(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        wheelFrontLeft.setPower(frontLeftSpeed);
        wheelFrontRight.setPower(frontRightSpeed);
        wheelBackLeft.setPower(backLeftSpeed);
        wheelBackRight.setPower(backRightSpeed);
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

    private void stopAllMotors() {
        setPower(0, 0, 0, 0);
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
            moveCounts = (int) (distance * GyroDriveRobot.NERVEREST20_COUNTS_PER_INCH);

            // Set Target and Turn On RUN_TO_POSITION
            int frontLeftTarget = wheelFrontLeft.getCurrentPosition()+ moveCounts;
            int frontRightTarget = wheelFrontRight.getCurrentPosition() + moveCounts;
            int backLeftTarget = wheelBackLeft.getCurrentPosition() + moveCounts;
            int backRightTarget = wheelBackRight.getCurrentPosition() + moveCounts;

            runToTarget(frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            setPower(speed, speed, speed, speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    wheelBackLeft.isBusy() &&
                    wheelBackRight.isBusy() &&
                    wheelFrontLeft.isBusy() &&
                    wheelFrontRight.isBusy()) {


                // Use gyro to drive in a straight line.
                correction = checkDirection();
                leftSpeed = power - correction;
                rightSpeed = power + correction;

                setPower(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
                updateStrafeTelemetry(correction, frontLeftTarget, frontRightTarget, backLeftTarget, backRightTarget);
            }

            resetMotors();
        }
    }

    private void updateStrafeTelemetry(double correction, int frontLeftTarget, int frontRightTarget, int backLeftTarget, int backRightTarget) {
        opMode.telemetry.addData("Corr",  "%5.1f",  correction);
        opMode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d",
                frontLeftTarget,  frontRightTarget, backLeftTarget, backRightTarget);
        opMode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d",
                wheelFrontLeft.getCurrentPosition(),
                wheelFrontRight.getCurrentPosition(),
                wheelBackLeft.getCurrentPosition(),
                wheelBackRight.getCurrentPosition());
        opMode.telemetry.addData("Speed",   "%5.1f:%5.1f:%5.1f:%5.1f",
                wheelFrontLeft.getPower(), wheelFrontRight.getPower(),
                wheelBackLeft.getPower(), wheelBackRight.getPower());
        opMode.telemetry.update();
    }
}
