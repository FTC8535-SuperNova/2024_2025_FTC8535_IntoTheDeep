package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Main_Teleop extends LinearOpMode {

    //initialize motors and servos
    private DcMotor shoulder_motor_1 = null;
    private DcMotor shoulder_motor_2 = null;

    private DcMotor climber = null;

    private DcMotor linear_slide = null;

    private Servo Claw = null;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    @Override
    public void runOpMode() {
        //Connect motors to irl motors
        shoulder_motor_1 = hardwareMap.get(DcMotor.class, "shoulder_left");
        shoulder_motor_2 = hardwareMap.get(DcMotor.class, "shoulder_right");

        climber = hardwareMap.get(DcMotor.class, "Climber");

        linear_slide = hardwareMap.get(DcMotor.class, "Linear_slide");

        Claw = hardwareMap.get(Servo.class, "Claw");

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");


        //set motor directions
        shoulder_motor_1.setDirection(DcMotor.Direction.FORWARD);
        shoulder_motor_2.setDirection(DcMotor.Direction.REVERSE);

        climber.setDirection(DcMotor.Direction.FORWARD);

        linear_slide.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();

        //set servo pos
        double pos = 1.0;
        while (opModeIsActive()) {

            //reading controller inputs
            double shoulder_drive = -1 * gamepad2.left_stick_y;

            double climber_drive = -1 * gamepad2.right_trigger;

            double linear_slide_drive = 0.5 * gamepad2.right_stick_y;

            if (gamepad2.x) {
                pos = 1.0;
            } else if (gamepad2.y){
                pos = 0.5;
            }

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.right_stick_x;
            double yaw     =  gamepad1.left_stick_x;

            boolean isFastMode = false;

            if (gamepad1.right_trigger == 1) {
                isFastMode = true;
            } else {
                isFastMode = false;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = 0;
            double rightFrontPower = 0;
            double leftBackPower = 0;
            double rightBackPower = 0;
            if (isFastMode) {
                leftFrontPower = axial + lateral + yaw;
                rightFrontPower = axial - lateral - yaw;
                leftBackPower = axial - lateral + yaw;
                rightBackPower = axial + lateral - yaw;
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



            //assign power to motors
            shoulder_motor_1.setPower(shoulder_drive);
            shoulder_motor_2.setPower(shoulder_drive);

            climber.setPower(climber_drive);

            linear_slide.setPower(linear_slide_drive);

            Claw.setPosition(pos);

            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();

        }

    }
}
