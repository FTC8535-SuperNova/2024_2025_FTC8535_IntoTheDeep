package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Proportional Integral Derivative Controller
 */
public class PositionPIDController {

    public PositionPIDController(DcMotor motor, double kp, double ki, double kd) {
        this.motor = motor;
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
    }

    DcMotor motor;

    double Kp;
    double Ki;
    double Kd;

    double integralSum = 0;

    double lastError = 0;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    public double computeMotorPower(double desiredPos) {
        double currentPosition = motor.getCurrentPosition();

        // calculate the error
        double error = desiredPos - currentPosition;

        // rate of change of the error
        double derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        lastError = error;

        // reset the timer for next time
        timer.reset();

        return out;
    }

}
