package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class testEncoders extends LinearOpMode {

    // Initialize motors
    private DcMotor motor_test_left = null;
    private DcMotor motor_test_right = null;

    @Override
    public void runOpMode() {
        // Connect shoulder_motor variables to shoulder motors on robot
        motor_test_left = hardwareMap.get(DcMotor.class, "shoulder_left");
        motor_test_right = hardwareMap.get(DcMotor.class, "shoulder_right");

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
//    public void getShoulderTic(){
//        int Encoder_ticks_1 = shoulder_motor_1.getCurrentPosition()
//        int Encoder_ticks_2 = shoulder_motor_2.getCurrentPosition()
//        telemetry.addData("Encoder value", Encoder_ticks_1)
//        telemetry.addData("Encoder value", Encoder_ticks_2)
//
//        if (Encoder_ticks > 180 in Encoder_ticks){
//            shoulder_motor_1.setMode(DcMotor.RunMode.RUN_TO_POSITION)
//            shoulder_motor_2.setMode(DcMotor)
//        }
//    }
}