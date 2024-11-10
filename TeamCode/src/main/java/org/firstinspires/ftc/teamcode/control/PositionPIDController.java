package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Proportional Integral Derivative Controller
 */
public class PositionPIDController {

    public PositionPIDController(DcMotor motor) {
        this.motor = motor;
    }

    DcMotor motor;

    double Kp = 0.0005;
    double Ki = 0.0001;
    double Kd = 0.00001;

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
