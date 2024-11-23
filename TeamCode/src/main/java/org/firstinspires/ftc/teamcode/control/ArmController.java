package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {

    final int SHOULDER_MAX_LIMIT = 1520;
    final int SHOULDER_BOUNDARY_LIMIT = 700; //TODO: Get Encoder value (placeholder here)
    int shoulder_min_limit = 0;
    final double SHOULDER_KP = 0.002;
    final double SHOULDER_KI = 0.0001;
    final double SHOULDER_KD = 0.00001;

    final int LINEAR_SLIDE_MAX_LIMIT = 3100;
    int lin_slide_max_limit = LINEAR_SLIDE_MAX_LIMIT;
    final int LINEAR_SLIDE_HORIZONTAL_LIMIT = 2000; //TODO: Get Encoder value (placeholder here)
    final double LINEAR_SLIDE_KP = 0.005;
    final double LINEAR_SLIDE_KI = 0.00;
    final double LINEAR_SLIDE_KD = 0.00;
    
    double desiredShoulderPos = 0;
    double desiredLinearSlidePos = 0;

    DcMotor shoulder_motor_1;
    DcMotor shoulder_motor_2;
    DcMotor linear_slide_motor;

    PositionPIDController shoulderPID1;
    PositionPIDController shoulderPID2;
    PositionPIDController linearSlidePID;

    public void init(DcMotor shoulder_motor_1, DcMotor shoulder_motor_2, DcMotor linear_slide_motor) {
        // Connect shoulder_motor variables to shoulder motors on robot
        // Initialize motors
        this.shoulder_motor_1 = shoulder_motor_1;
        this.shoulder_motor_2 = shoulder_motor_2;
        this.linear_slide_motor = linear_slide_motor;

        // Set motor directions
        shoulder_motor_1.setDirection(DcMotor.Direction.REVERSE);
        shoulder_motor_2.setDirection(DcMotor.Direction.FORWARD);
        linear_slide_motor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to use encoder mode
        shoulder_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shoulder_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoulderPID1 = new PositionPIDController(shoulder_motor_1,
                SHOULDER_KP, SHOULDER_KI, SHOULDER_KD);
        shoulderPID2 = new PositionPIDController(shoulder_motor_2,
                SHOULDER_KP, SHOULDER_KI, SHOULDER_KD);
        linearSlidePID = new PositionPIDController(linear_slide_motor,
                LINEAR_SLIDE_KP, LINEAR_SLIDE_KI, LINEAR_SLIDE_KD);
    }

    public void update(double shoulderCommand, double linearSlideCommand, Telemetry telemetry) {
        desiredShoulderPos += 10 * shoulderCommand;
        desiredLinearSlidePos += 25 * linearSlideCommand;

        if (desiredLinearSlidePos > LINEAR_SLIDE_HORIZONTAL_LIMIT) {
            shoulder_min_limit = SHOULDER_BOUNDARY_LIMIT;
        } else {
            shoulder_min_limit = 0;
        }
        
        //limits the shoulder movement
        if (desiredShoulderPos > SHOULDER_MAX_LIMIT) {
            desiredShoulderPos = SHOULDER_MAX_LIMIT;
        } else if (desiredShoulderPos < shoulder_min_limit){
            desiredShoulderPos = shoulder_min_limit;
        }

        //limits linear_slide movement
        if (desiredShoulderPos > SHOULDER_BOUNDARY_LIMIT) {
           lin_slide_max_limit = LINEAR_SLIDE_MAX_LIMIT;
        } else if (desiredShoulderPos < SHOULDER_BOUNDARY_LIMIT) {
            lin_slide_max_limit = LINEAR_SLIDE_HORIZONTAL_LIMIT;
        }

        if (desiredLinearSlidePos > lin_slide_max_limit) {
            desiredLinearSlidePos = lin_slide_max_limit;
        } else if (desiredLinearSlidePos < 0) {
            desiredLinearSlidePos = 0;
        }

        double shoulderPower1 = shoulderPID1.computeMotorPower(desiredShoulderPos);
        double shoulderPower2 = shoulderPID2.computeMotorPower(desiredShoulderPos);
        double linearSlidePower = linearSlidePID.computeMotorPower(desiredLinearSlidePos);
        
        // Optional: Display telemetry data for debugging
        telemetry.addData("Shoulder Motor 1 Position", shoulder_motor_1.getCurrentPosition());
        telemetry.addData("Shoulder Motor 2 Position", shoulder_motor_2.getCurrentPosition());
        telemetry.addData("Shoulder Desired Pos", desiredShoulderPos);
        telemetry.addData("Shoulder 1 Power", shoulderPower1);
        telemetry.addData("Shoulder 2 Power", shoulderPower2);
        telemetry.addData("Linear Slide Position", linear_slide_motor.getCurrentPosition());
        telemetry.addData("Linear Slide Desired Pos", desiredLinearSlidePos);
        telemetry.addData("Linear Slide Power", linearSlidePower);

        shoulder_motor_1.setPower(shoulderPower1);
        shoulder_motor_2.setPower(shoulderPower2);
        linear_slide_motor.setPower(linearSlidePower);
    }

    public void ZeroLSEncoders() {
        linear_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}