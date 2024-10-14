package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name = "Variable Intake", group = "Test")
@Disabled
public class VariableIntake_Test extends LinearOpMode{
    // Declare Motors
    private DcMotor right_intake = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        right_intake = hardwareMap.get(DcMotor.class, "right_intake");
        waitForStart();

        while (opModeIsActive()) {
            double myMotorPower;
            //Push Pixels Out
            if (gamepad1.dpad_up==true) {
                double IntakePower = gamepad1.left_trigger;
                //myMotorPower = Range.clip(IntakePower, -1.0, 1.0);
                myMotorPower = Range.clip(IntakePower, -0.5, 0.5);
                right_intake.setDirection(DcMotor.Direction.REVERSE);
                right_intake.setPower(myMotorPower);
            }
            //Pull Pixels In
            else if(gamepad1.dpad_down) {
                double OuttakePower = -gamepad1.right_trigger; //instead of neg, set it in FORWARD
                //myMotorPower = Range.clip(OuttakePower, -1.0, 1.0);
                myMotorPower = Range.clip(OuttakePower, -0.5, 0.5);
                right_intake.setDirection(DcMotor.Direction.REVERSE);
                right_intake.setPower(myMotorPower);
                telemetry.addData("right_intake", "Forward/Backwards Power (%.2f)", myMotorPower);
                telemetry.update();
            }
            else {
                right_intake.setPower(0.0);
            }

        }
    }
}
