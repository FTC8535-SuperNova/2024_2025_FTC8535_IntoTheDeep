package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@TeleOp (name = "Variable Linear Slide", group = "Test")
@Disabled
public class VariableLinearSlide_Test extends LinearOpMode {
    //declaring our variables,initialize our motors
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;
    @Override
    public void runOpMode() throws InterruptedException {
        //initialize our telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //add motors to config map
        leftSlide = hardwareMap.get(DcMotor.class, "left_linear_slide");
        rightSlide = hardwareMap.get(DcMotor.class, "right_linear_slide");
        leftSlide.setDirection(DcMotor.Direction.FORWARD);
        rightSlide.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double myMotorPower;

            double drivePower = gamepad2.right_stick_y; //instead of neg, set dir REVERSE
            myMotorPower = Range.clip(drivePower, -1.0, 1.0);

            leftSlide.setPower(myMotorPower);
            rightSlide.setPower(myMotorPower);
            telemetry.addData("myMotor", "Forward/Backwards Power (%.2f)", myMotorPower);
            telemetry.update();
        }
    }
}
