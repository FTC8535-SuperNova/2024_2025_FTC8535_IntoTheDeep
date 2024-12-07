package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {

    static final int SHOULDER_POS_INCREMENT = 20;
    static final int LINEAR_SLIDE_POS_INCREMENT = 35;
    public static final int GRAB_SPECIMEN_LINEAR_SLIDE_POS = 410;
    public static final int GRAB_SPECIMEN_SHOULDER_POS = 500;
    public static final int HIGH_SPECIMEN_LINEAR_SLIDE_POS = 100;
    public static final int HIGH_SPECIMEN_SHOULDER_POS = 1200;
    public static final int HIGH_BASKET_LINEAR_SLIDE_POS = 2700;
    public static final int HIGH_BASKET_SHOULDER_POS = 1425;

    final int SHOULDER_MAX_LIMIT = 1600;
    final int SHOULDER_BOUNDARY_LIMIT = 1000;
    int shoulder_min_limit = 0;
    final double SHOULDER_KP = 0.0025;
    final double SHOULDER_KI = 0.0001;
    final double SHOULDER_KD = 0.00001;

    final int LINEAR_SLIDE_MAX_LIMIT = 3100;
    int lin_slide_max_limit = LINEAR_SLIDE_MAX_LIMIT;
    final int LINEAR_SLIDE_HORIZONTAL_LIMIT = 2500;
    final double LINEAR_SLIDE_KP = 0.0045;
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

    private ArmMode armMode = ArmMode.DRIVER_CONTROL;

    public void init(DcMotor shoulder_motor_1, DcMotor shoulder_motor_2, DcMotor linear_slide_motor, boolean zeroEncoders) {
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
        if (zeroEncoders) {
            shoulder_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulder_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linear_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        shoulder_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linear_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shoulder_motor_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoulder_motor_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shoulderPID1 = new PositionPIDController(shoulder_motor_1,
                SHOULDER_KP, SHOULDER_KI, SHOULDER_KD);
        shoulderPID2 = new PositionPIDController(shoulder_motor_2,
                SHOULDER_KP, SHOULDER_KI, SHOULDER_KD);
        linearSlidePID = new PositionPIDController(linear_slide_motor,
                LINEAR_SLIDE_KP, LINEAR_SLIDE_KI, LINEAR_SLIDE_KD);
    }

    public void update(double shoulderCommand, double linearSlideCommand, boolean overrideArmLowLimits, Telemetry telemetry) {
        boolean shoulderInPos = false;
        boolean linSlideInPos = false;
        switch (armMode) {
            case GRAB_SPECIMEN:
                if (Math.abs(GRAB_SPECIMEN_SHOULDER_POS - desiredShoulderPos) <= SHOULDER_POS_INCREMENT) {
                    desiredShoulderPos = GRAB_SPECIMEN_SHOULDER_POS;
                    shoulderInPos = true;
                } else if (desiredShoulderPos < GRAB_SPECIMEN_SHOULDER_POS) {
                    desiredShoulderPos += SHOULDER_POS_INCREMENT;
                } else {
                    desiredShoulderPos -= SHOULDER_POS_INCREMENT;
                }
                break;
            case HIGH_SPECIMEN:
                if (Math.abs(HIGH_SPECIMEN_SHOULDER_POS - desiredShoulderPos) < SHOULDER_POS_INCREMENT) {
                    desiredShoulderPos = HIGH_SPECIMEN_SHOULDER_POS;
                    shoulderInPos = true;
                } else if (desiredShoulderPos < HIGH_SPECIMEN_SHOULDER_POS) {
                    desiredShoulderPos += SHOULDER_POS_INCREMENT;
                } else {
                    desiredShoulderPos -= SHOULDER_POS_INCREMENT;
                }
                break;
            case HIGH_BASKET:
                if (Math.abs(HIGH_BASKET_SHOULDER_POS - desiredShoulderPos) < SHOULDER_POS_INCREMENT) {
                    desiredShoulderPos = HIGH_BASKET_SHOULDER_POS;
                    shoulderInPos = true;
                } else if (desiredShoulderPos < HIGH_BASKET_SHOULDER_POS) {
                    desiredShoulderPos += SHOULDER_POS_INCREMENT;
                } else {
                    desiredShoulderPos -= SHOULDER_POS_INCREMENT;
                }
            default:
                desiredShoulderPos += SHOULDER_POS_INCREMENT * shoulderCommand;
                break;
        }

        if (desiredLinearSlidePos > LINEAR_SLIDE_HORIZONTAL_LIMIT) {
            shoulder_min_limit = SHOULDER_BOUNDARY_LIMIT;
        } else if (overrideArmLowLimits) {
            shoulder_min_limit = -1000;
        } else {
            shoulder_min_limit = 0;
        }

        switch (armMode) {
            case GRAB_SPECIMEN:
                if (Math.abs(GRAB_SPECIMEN_LINEAR_SLIDE_POS - desiredLinearSlidePos) <= LINEAR_SLIDE_POS_INCREMENT) {
                    desiredLinearSlidePos = GRAB_SPECIMEN_LINEAR_SLIDE_POS;
                    linSlideInPos = true;
                } else if (desiredLinearSlidePos < GRAB_SPECIMEN_LINEAR_SLIDE_POS) {
                    desiredLinearSlidePos += LINEAR_SLIDE_POS_INCREMENT;
                } else {
                    desiredLinearSlidePos -= LINEAR_SLIDE_POS_INCREMENT;
                }
                break;
            case HIGH_SPECIMEN:
                if (Math.abs(HIGH_SPECIMEN_LINEAR_SLIDE_POS - desiredLinearSlidePos) < LINEAR_SLIDE_POS_INCREMENT) {
                    desiredLinearSlidePos = HIGH_SPECIMEN_LINEAR_SLIDE_POS;
                    linSlideInPos = true;
                } else if (desiredLinearSlidePos < HIGH_SPECIMEN_LINEAR_SLIDE_POS) {
                    desiredLinearSlidePos += LINEAR_SLIDE_POS_INCREMENT;
                } else {
                    desiredLinearSlidePos -= LINEAR_SLIDE_POS_INCREMENT;
                }
                break;
            case HIGH_BASKET:
                if (Math.abs(HIGH_BASKET_LINEAR_SLIDE_POS - desiredLinearSlidePos) < LINEAR_SLIDE_POS_INCREMENT) {
                    desiredLinearSlidePos = HIGH_BASKET_LINEAR_SLIDE_POS;
                    linSlideInPos = true;
                } else if (desiredLinearSlidePos < HIGH_BASKET_LINEAR_SLIDE_POS) {
                    desiredLinearSlidePos += LINEAR_SLIDE_POS_INCREMENT;
                } else {
                    desiredLinearSlidePos -= LINEAR_SLIDE_POS_INCREMENT;
                }
                break;
            default:
                desiredLinearSlidePos += LINEAR_SLIDE_POS_INCREMENT * linearSlideCommand;
                break;
        }
        if (armMode != ArmMode.DRIVER_CONTROL && shoulderInPos && linSlideInPos){
            armMode = ArmMode.DRIVER_CONTROL;
        }

        //limits the shoulder movement
        if (desiredShoulderPos > SHOULDER_MAX_LIMIT) {
            desiredShoulderPos = SHOULDER_MAX_LIMIT;
        } else if (desiredShoulderPos < shoulder_min_limit) {
            desiredShoulderPos = shoulder_min_limit;
        }

        //limits linear_slide movement
        if (desiredShoulderPos > SHOULDER_BOUNDARY_LIMIT) {
           lin_slide_max_limit = LINEAR_SLIDE_MAX_LIMIT;
        } else if (desiredShoulderPos < SHOULDER_BOUNDARY_LIMIT) {
            lin_slide_max_limit = LINEAR_SLIDE_HORIZONTAL_LIMIT;
        }

        int linear_slide_min_limit = overrideArmLowLimits ? -2000 : 0;
        if (desiredLinearSlidePos > lin_slide_max_limit) {
            desiredLinearSlidePos = lin_slide_max_limit;
        } else if (desiredLinearSlidePos < linear_slide_min_limit) {
            desiredLinearSlidePos = linear_slide_min_limit;
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

    public void zeroLSEncoders() {
        linear_slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_slide_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void zeroShoulderEncoders() {
        shoulder_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder_motor_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoulder_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArmMode(ArmMode armMode) {
        this.armMode = armMode;
    }

}