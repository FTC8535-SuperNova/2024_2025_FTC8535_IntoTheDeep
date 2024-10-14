package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous (name="Autonomous A2 (Blue)", group = "Test")

public class AutonomousA2Blue extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        // Drive
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        // Intake
        DcMotor rightIntake = hardwareMap.get(DcMotor.class, "right_intake");

        // Set Motor Direction and Power
        // Drive
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        // Intake
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // *** Drive Code Start

            // Drive Forward (1s = 1000 ms)
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(100);

            //Stop
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(500);

            //Turn Left
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(1000);

            //Turn Right
            //leftFrontDrive.setPower(0.5);
            //leftBackDrive.setPower(0.5);
            //rightFrontDrive.setPower(-0.5);
            //rightBackDrive.setPower(-0.5);
            //sleep(1000);

            //Stop
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(1000);

            // Drive Forward
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(4000);

            //Stop
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            //Strafe Left
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            sleep(200);

            //Strafe Right
            //leftFrontDrive.setPower(0.5);
            //leftBackDrive.setPower(-0.5);
            //rightFrontDrive.setPower(-0.5);
            //rightBackDrive.setPower(0.5);
            //sleep(200);

           //Stop
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(500);

            //Push Yellow Pixel Out
            rightIntake.setPower(0.35);
            sleep(1000);
            rightIntake.setPower(0);

            //Stop - Parking End
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            sleep(20700);

            // Show the elapsed game time.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
    }
}