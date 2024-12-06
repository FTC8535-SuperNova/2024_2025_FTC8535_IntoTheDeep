package org.firstinspires.ftc.teamcode.control;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotController {
    private Telemetry telemetry;

    //initialize motors and servos
    private DcMotor shoulder_motor_1 = null;
    private DcMotor shoulder_motor_2 = null;

    private DcMotor climber = null;

    private DcMotor linear_slide = null;

    private Servo claw = null;

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private final ArmController armController = new ArmController();

    public void init(HardwareMap hardwareMap, Telemetry telemetry, boolean zeroEncoders) {
        this.telemetry = telemetry;
        //Connect motors to irl motors
        connectMotorsToHub(hardwareMap);

        //set motor directions
        setMotorDirections(zeroEncoders);

        armController.init(shoulder_motor_1, shoulder_motor_2, linear_slide, zeroEncoders);

    }

    public void update(double shoulderCommand, double linearSlideCommand, double climberDrive, double axial, double lateral,
                       double yaw, boolean isFastMode, boolean clawClosed, boolean zeroLinearSlide, boolean zeroShoulder, boolean overrideArmLowLimits) {

        //reading controller inputs
        telemetry.addData("Shoulder Cmd", shoulderCommand);
        telemetry.addData("Lin Slide Cmd", linearSlideCommand);
        armController.update(shoulderCommand, linearSlideCommand, overrideArmLowLimits, telemetry);

        double clawPos = clawClosed ? 1.0 : 0.5;

        if (zeroLinearSlide) {
            armController.zeroLSEncoders();
        }
        if (zeroShoulder) {
            armController.zeroShoulderEncoders();
        }

        WheelPower wheelPower = computeWheelPower(axial, lateral, yaw, isFastMode);

        //assign power to motors
        assignMotorPowers(climberDrive, clawPos, wheelPower);

        // Show the elapsed game time and wheel power.
        updateTelemetry(wheelPower, telemetry);

    }

    private void updateTelemetry(WheelPower wheelPower, Telemetry telemetry) {
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", wheelPower.leftFrontPower, wheelPower.rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", wheelPower.leftBackPower, wheelPower.rightBackPower);
        telemetry.update();
    }

    private void assignMotorPowers(double climber_drive, double clawPos, WheelPower wheelPower) {
        climber.setPower(climber_drive);

        claw.setPosition(clawPos);

        leftFrontDrive.setPower(wheelPower.leftFrontPower);
        rightFrontDrive.setPower(wheelPower.rightFrontPower);
        leftBackDrive.setPower(wheelPower.leftBackPower);
        rightBackDrive.setPower(wheelPower.rightBackPower);
    }

    @NonNull
    private WheelPower computeWheelPower(double axial, double lateral,
                                         double yaw, boolean isFastMode) {
        double max;

        /*
         Combine the joystick requests for each axis-motion to determine each wheel's power.
         Set up a variable for each drive wheel to save the power level for telemetry.
        */
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;
        if (isFastMode) {
            leftFrontPower = (axial + lateral + yaw)*3/4;
            rightFrontPower = (axial - lateral - yaw)*3/4;
            leftBackPower = (axial - lateral + yaw)*3/4;
            rightBackPower = (axial + lateral - yaw)*3/4;
        } else {
            leftFrontPower  = (axial + lateral + yaw)/4;
            rightFrontPower = (axial - lateral - yaw)/4;
            leftBackPower   = (axial - lateral + yaw)/4;
            rightBackPower  = (axial + lateral - yaw)/4;
        }

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        return new WheelPower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private static class WheelPower {
        public final double leftFrontPower;
        public final double rightFrontPower;
        public final double leftBackPower;
        public final double rightBackPower;

        public WheelPower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
            this.leftFrontPower = leftFrontPower;
            this.rightFrontPower = rightFrontPower;
            this.leftBackPower = leftBackPower;
            this.rightBackPower = rightBackPower;
        }
    }

    private void setMotorDirections(boolean zeroEncoders) {
        climber.setDirection(DcMotor.Direction.FORWARD);
        if (zeroEncoders) {
            climber.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        climber.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        if (zeroEncoders) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void connectMotorsToHub(HardwareMap hardwareMap) {
        shoulder_motor_1 = hardwareMap.get(DcMotor.class, "shoulder_left");
        shoulder_motor_2 = hardwareMap.get(DcMotor.class, "shoulder_right");

        climber = hardwareMap.get(DcMotor.class, "climber");

        linear_slide = hardwareMap.get(DcMotor.class, "linear_slide");

        claw = hardwareMap.get(Servo.class, "claw");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
    }


    public void setArmMode(ArmMode armMode) {
        armController.setArmMode(armMode);
    }
}
