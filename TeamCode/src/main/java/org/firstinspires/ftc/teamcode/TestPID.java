package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.control.PositionPIDController;

@TeleOp
public class TestPID extends LinearOpMode {

    final int SHOULDER_ENCODER_LIMIT = 1520;

    double desiredShoulderPos = 0;

    @Override
    public void runOpMode() {
        // Connect shoulder_motor variables to shoulder motors on robot
        // Initialize motors
        DcMotor shoulder_motor_1 = hardwareMap.get(DcMotor.class, "shoulder_left");
        DcMotor shoulder_motor_2 = hardwareMap.get(DcMotor.class, "shoulder_right");

        // Set motor directions
        shoulder_motor_1.setDirection(DcMotor.Direction.FORWARD);
        shoulder_motor_2.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to use encoder mode
        shoulder_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PositionPIDController shoulderPID1 = new PositionPIDController(shoulder_motor_1);
        PositionPIDController shoulderPID2 = new PositionPIDController(shoulder_motor_2);

        waitForStart();

        while (opModeIsActive()) {

            double shoulderCommand = -1 * gamepad2.left_stick_y;
            desiredShoulderPos += 10 * shoulderCommand;

            if (desiredShoulderPos > SHOULDER_ENCODER_LIMIT) {
                desiredShoulderPos = SHOULDER_ENCODER_LIMIT;
            } else if (desiredShoulderPos < 0) {
                desiredShoulderPos = 0;
            }

            double shoulderPower1 = shoulderPID1.computeMotorPower(desiredShoulderPos);
            double shoulderPower2 = shoulderPID2.computeMotorPower(desiredShoulderPos);

            // Optional: Display telemetry data for debugging
            telemetry.addData("Shoulder Motor 1 Position", shoulder_motor_1.getCurrentPosition());
            telemetry.addData("Shoulder Motor 2 Position", shoulder_motor_2.getCurrentPosition());
            telemetry.addData("Shoulder Desired Pos", desiredShoulderPos);
            telemetry.addData("Shoulder 1 Power", shoulderPower1);
            telemetry.addData("Shoulder 2 Power", shoulderPower2);
            telemetry.update();

            shoulder_motor_1.setPower(shoulderPower1);
            shoulder_motor_2.setPower(shoulderPower2);
        }

        // Stop the motors when the OpMode is stopped
//        shoulder_motor_1.setPower(0);
//        shoulder_motor_2.setPower(0);
    }
}