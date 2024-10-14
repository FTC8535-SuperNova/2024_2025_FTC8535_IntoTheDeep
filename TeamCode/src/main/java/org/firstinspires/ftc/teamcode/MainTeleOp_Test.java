package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "MainTeleOp Routine - Test Version", group = "Test")
@Disabled
public class MainTeleOp_Test extends LinearOpMode{
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        /* Drive
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Intake
        DcMotor rightIntake = hardwareMap.get(DcMotor.class, "right_intake");
        // Linear Slide
        DcMotor leftSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        DcMotor rightSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");*/
        // Pixel Drop (Outtake Servo)
        Servo outServo = hardwareMap.servo.get("out_servo");
        // Drone Launch (Drone Servo)
        Servo droneServo = hardwareMap.servo.get("drone_servo");
        /* Hanging
        DcMotor hangMotor = hardwareMap.get(DcMotor.class, "hang_end");

        // Set Motor Direction and Power
        // Drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Intake
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        // Linear Slide
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);
        // Hanging
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE); */

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /* *** Drive Code Start

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

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

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

            // Drive Code End ***

            // *** Intake (Pixel Pull In and Push Out) Code Start
            double inMotorPower;

            // Push Pixels Out
            if (gamepad1.dpad_up) {
                double ppoPower = gamepad1.left_trigger;
                inMotorPower = Range.clip(ppoPower, -1.0, 1.0);
                rightIntake.setDirection(DcMotor.Direction.REVERSE);
                rightIntake.setPower(inMotorPower);

                telemetry.addData("Intake Motor", "Forwards Power (%.2f)", inMotorPower);
                telemetry.update();
            }
            // Pull Pixels In
            else if(gamepad1.dpad_down) {
                double ppiPower = gamepad1.right_trigger; //instead of neg, set it in FORWARD
                inMotorPower = Range.clip(ppiPower, -1.0, 1.0);
                rightIntake.setDirection(DcMotor.Direction.FORWARD);
                rightIntake.setPower(inMotorPower);

                telemetry.addData("Intake Motor", "Backwards Power (%.2f)", inMotorPower);
                telemetry.update();
            }
            else {
                rightIntake.setPower(0.0);
            }

            // Intake Code End ***

            // *** Linear Slide Code Start

            double lsMotorPower;

            double drivePower = gamepad2.right_stick_y; //instead of neg, set dir REVERSE
            lsMotorPower = Range.clip(drivePower, -1.0, 1.0);

            leftSlide.setPower(lsMotorPower);
            rightSlide.setPower(lsMotorPower);
            telemetry.addData("Linear Slide Motor", "Forward/Backwards Power (%.2f)", lsMotorPower);
            telemetry.update();

            // Linear Slide Code End ***

            // *** Pixel Drop Code Start

            // check to see if we need to move the servo. */
            if (gamepad2.x) {
                // move to 0 degrees (close)
                outServo.setPosition(0.08);
            } else if (gamepad2.y) {
                //move to 30 degrees.
                outServo.setPosition(0.17);
            } /*else if (gamepad2.b) {
                //move to 45 degrees.
                outServo.setPosition(0.25);
            } else if (gamepad2.a) {
                // move to 180 degrees.
                //outServo.setPosition(1);
            }*/

            telemetry.addData("Servo Position", outServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            // Pixel Drop Code End ***

            // *** Drone Launch Code Start

            // check to see if we need to move the servo.
            if (gamepad2.x) {
                // move to 0 degrees (close).
                droneServo.setPosition(0);
            } else if (gamepad2.y) {
                //move to 45 degrees (*find the correct angle*).
                droneServo.setPosition(0.25);
            }

            telemetry.addData("Servo Position", droneServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            /* Drone Launch Code End ***

            // *** Hanging Code
            boolean dpadUp = gamepad2.dpad_up;
            boolean dpadDown = gamepad2.dpad_down;

            if (dpadUp){
                // motor in reverse direction to extend linear slide
                hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                hangMotor.setPower(1.0);
            }
            else if (dpadDown){
                // motor in forward direction to retract linear slide
                hangMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                hangMotor.setPower(1.0);
            }
            else {
                // otherwise switch it off
                hangMotor.setPower(0.0);
            }

            telemetry.addData("Dpad Up",dpadUp);
            telemetry.addData("Dpad Down",dpadDown);
            telemetry.addData("Hanging Motor Speed", hangMotor.getPower());
            telemetry.update();

            // Hanging Code End *** */
        }
    }
}
