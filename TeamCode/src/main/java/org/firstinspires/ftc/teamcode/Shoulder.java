package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Shoulder extends LinearOpMode {

    // Initialize motors
    private DcMotor shoulder_motor_1 = null;
    private DcMotor shoulder_motor_2 = null;
    private boolean holdingPosition = false; // Flag to check if we're in holding mode
    private int holdPosition1, holdPosition2; // Variables to store hold positions

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

            // Use gamepad2 joystick to control shoulder motors
            double drive = -1 * gamepad2.left_stick_y;

            // If the joystick is being moved significantly
            if (Math.abs(drive) > 0.1) {
                holdingPosition = false; // Not holding anymore
                shoulder_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shoulder_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // Slow down descent if moving down
                if (drive < 0) {
                    drive /= 4;
                }

                // Assign power to motors
                shoulder_motor_1.setPower(drive);
                shoulder_motor_2.setPower(drive);
            }
            else {
                // If the joystick is released, hold the current position
                if (!holdingPosition) {
                    holdingPosition = true;

                    // Set hold positions based on current positions
                    holdPosition1 = shoulder_motor_1.getCurrentPosition();
                    holdPosition2 = shoulder_motor_2.getCurrentPosition();

                    // Set target positions for holding
                    shoulder_motor_1.setTargetPosition(holdPosition1);
                    shoulder_motor_2.setTargetPosition(holdPosition2);

                    // Switch to RUN_TO_POSITION mode for holding
                    shoulder_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    shoulder_motor_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // Set power to hold position
                    shoulder_motor_1.setPower(0.5);
                    shoulder_motor_2.setPower(0.5);
                }
            }

            // Optional: Display telemetry data for debugging
            telemetry.addData("Shoulder Motor 1 Position", shoulder_motor_1.getCurrentPosition());
            telemetry.addData("Shoulder Motor 2 Position", shoulder_motor_2.getCurrentPosition());
            telemetry.addData("Joystick Y", drive);
            telemetry.update();
        }

        // Stop the motors when the OpMode is stopped
        shoulder_motor_1.setPower(0);
        shoulder_motor_2.setPower(0);
    }
}
