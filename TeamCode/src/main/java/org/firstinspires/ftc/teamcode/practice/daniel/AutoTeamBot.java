package org.firstinspires.ftc.teamcode.practice.daniel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * This is our team robot, with all the functions necessary to run the Teleop in ManualTeleop.
 */

public class AutoTeamBot {
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

    public AutoTeamBot(HardwareMap hardwareMap) {
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
    }

    public void move(int distance, int scale) throws InterruptedException{
        wheelFrontLeft.setPower(scale);
        wheelFrontRight.setPower(scale);
        wheelBackLeft.setPower(scale);
        wheelBackRight.setPower(scale);

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
     * Flap the ring stabilizer to help to lay the ring in the magazine
     *
     * @throws InterruptedException
     */


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
    public void openGrabber() {
        grabber.setPosition(1);
    }
    public void closeGrabber() {
        grabber.setPosition(0);
    }


    /**
     * lifting the arm that will hold the wobble goal so that it can clear the perimeter
     */
    public void liftArm() {
        arm.setPower(0.7);
    }
    public void dropArm() {
        arm.setPower(-0.7);
    }
    public void stopArm() {
        arm.setPower(0);
    }

    /**
     * flywheel for launcher
     */

    public void shoot(double distance) { flywheel.setPower(distance); }
    public void stopShoot() { flywheel.setPower(0); }
}
