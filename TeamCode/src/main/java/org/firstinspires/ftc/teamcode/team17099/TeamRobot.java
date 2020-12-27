package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

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

    // Macanum drive speed control with turbo between 0.1 and 1.0, there are face and slow paces
    // for turbo adjustment
    private double turbo = 0.5;
    private double fastPace = 0.2;
    private double slowPace = 0.1;
    private boolean isFastPace = true;

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
    }

    /**
     * Update turbo change pace between slow and fast alternatively
     */
    public void flipPace() {
        this.isFastPace = !isFastPace;
    }

    /**
     * Increases or decrease drive train turbo. Use higher turbo for fast speed and slower turbo
     * for better maneuver
     *
     * @param increasing
     */
    public void updateTurbo(boolean increasing) {
        double pace = isFastPace ? fastPace : slowPace;
        if (increasing) {
            turbo = Math.min(0.5, turbo + pace);
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
        double lx = gamepad.left_stick_x;
        double ly = gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        double wheelFrontRightPower = turbo * (-lx - rx - ly);
        double wheelBackRightPower = turbo * (lx - rx - ly);
        double wheelFrontLeftPower = turbo * (lx + rx - ly);
        double wheelBackLeftPower = turbo * (-lx + rx - ly);

        wheelFrontLeft.setPower(wheelFrontLeftPower);
        wheelFrontRight.setPower(wheelFrontRightPower);
        wheelBackLeft.setPower(wheelBackLeftPower);
        wheelBackRight.setPower(wheelBackRightPower);
    }

    /**
     * Intake rings with pasta roller and pulley.
     */
    public void inTake() {
        intake.setPower(1.00);
        conveyor.setPower(1.00);
    }

    /**
     * Outtake rings in case ring is stuck or out of place
     */
    public void outTake() {
        intake.setPower(-1.00);
        conveyor.setPower(-1.00);
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
    public void stabilizeRing() throws InterruptedException {
        stabilizer.setPosition(0);
        TimeUnit.MILLISECONDS.sleep(1500);
        stabilizer.setPosition(1);
    }

    /**
     * pushing the ring into the launcher
     *
     * @throws InterruptedException
     */
    public void pushRing() throws InterruptedException {
        pusher.setPosition(1);
        TimeUnit.MILLISECONDS.sleep(500);
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
        TimeUnit.MILLISECONDS.sleep(500);
    }

    /**
     * lifting the arm that will hold the wobble goal so that it can clear the perimeter
     */
    public void liftArm() {
        arm.setPower(1);
    }
    public void dropArm() {
        arm.setPower(-1);
    }
    public void stopArm() {
        arm.setPower(0);
    }
    /**
     * flywheel for launcher
     */
    public void startFlywheel() {
        flywheel.setPower(1.00);
    }
    public void stopFlywheel() {
        flywheel.setPower(0);
    }
}
