package org.firstinspires.ftc.teamcode.team17099;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeamRobot {
    private HardwareMap hardwareMap;

    public DcMotor wheelFrontLeft = null;
    public DcMotor wheelFrontRight = null;
    public DcMotor wheelBackLeft = null;
    public DcMotor wheelBackRight = null;

    private double fastPace = 0.2;
    private double slowPace = 0.1;
    private double turbo = 0.5;
    private boolean isFastPace = true;

    public TeamRobot(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
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
    }

    public void flipPace() {
        this.isFastPace = !isFastPace;
    }

    public void updateTurbo(boolean increasing) {
        double pace = isFastPace ? fastPace : slowPace;
        if (increasing) {
            turbo = Math.min(1, turbo + pace);
        } else {
            turbo = Math.max(0.1, turbo - pace);
        }
    }

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
}
