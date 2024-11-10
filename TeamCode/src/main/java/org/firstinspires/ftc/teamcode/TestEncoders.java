package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestEncoders extends LinearOpMode {

    final int SHOULDER_ENCODER_LIMIT = 1520;

    // Initialize motors
    private DcMotor shoulder_motor_1 = null;
    private DcMotor shoulder_motor_2 = null;

    @Override
    public void runOpMode() {
        // Connect shoulder_motor variables to shoulder motors on robot
        shoulder_motor_1 = hardwareMap.get(DcMotor.class, "shoulder_left");
        shoulder_motor_2 = hardwareMap.get(DcMotor.class, "shoulder_right");

        // Set motor directions
        shoulder_motor_1.setDirection(DcMotor.Direction.FORWARD);
        shoulder_motor_2.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to use encoder mode
        shoulder_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            int shoulderEncoderPos = shoulder_motor_1.getCurrentPosition();
            double shoulderCommand = -1 * gamepad2.left_stick_y;

            if (shoulderEncoderPos > SHOULDER_ENCODER_LIMIT && shoulderCommand > 0) {
                shoulderCommand = 0;
            }

            // Optional: Display telemetry data for debugging
            telemetry.addData("Shoulder Motor 1 Position", shoulder_motor_1.getCurrentPosition());
            telemetry.addData("Shoulder Motor 2 Position", shoulder_motor_2.getCurrentPosition());
            telemetry.addData("Shoulder Command", shoulderCommand);
            telemetry.update();

            shoulder_motor_1.setPower(shoulderCommand);
            shoulder_motor_2.setPower(shoulderCommand);
        }

        // Stop the motors when the OpMode is stopped
//        shoulder_motor_1.setPower(0);
//        shoulder_motor_2.setPower(0);
    }
}