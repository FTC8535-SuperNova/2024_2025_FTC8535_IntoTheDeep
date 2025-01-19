package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;

@Autonomous(name="OdomAutoHighBasketAscent", group="OdomAutonomous", preselectTeleOp = "Main_Teleop")
//@Disabled

public class OdomAutonomousHighBasketAscent extends LinearOpMode {
    RobotController robotController = new RobotController();
    private final ElapsedTime runtime = new ElapsedTime();

    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        RAISE_ARM_HIGH_BASKET,
        MOVE_TO_BASKET,
        OPEN_CLAW,
        TURN,
        STRAFE,
        TARGET_RUNG,
        EXTEND_ARM

    }

    static final Pose2D TARGET_HIGH_BASKET = new Pose2D(DistanceUnit.MM,1060,50,AngleUnit.DEGREES,0);
    static final Pose2D TURN = new Pose2D(DistanceUnit.MM,1000, 50, AngleUnit.DEGREES, -90);
    static final Pose2D TARGET_RUNG = new Pose2D(DistanceUnit.MM,1000, 1500, AngleUnit.DEGREES, 0);


    private double shoulderCommand = 0;
    private double linearSlideCmd = 0;
    private boolean clawClosed = true;

    @Override
    public void runOpMode() {
        robotController.init(hardwareMap, telemetry, true);
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        //nav.setXYCoefficients(0.02,0.002,0.0,DistanceUnit.MM,12);
        //nav.setYawCoefficients(1,0,0.0, AngleUnit.DEGREES,2);
        nav.setDriveType(DriveToPoint.DriveType.MECANUM);

        StateMachine stateMachine;
        stateMachine = StateMachine.WAITING_FOR_START;

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        while (opModeIsActive()) {
            robotController.updateOdometry();


            switch (stateMachine){
                case WAITING_FOR_START:
                    //the first step in the autonomous
                    stateMachine = StateMachine.RAISE_ARM_HIGH_BASKET;
                    runtime.reset();
                    break;
                case RAISE_ARM_HIGH_BASKET:
                    robotController.setArmMode(ArmMode.HIGH_BASKET);
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.MOVE_TO_BASKET;
                    }
                    break;
                case MOVE_TO_BASKET:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_HIGH_BASKET, 0.45, 0)){
                        stateMachine = StateMachine.OPEN_CLAW;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAW:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.TURN;
                        runtime.reset();
                    }
                    break;
                case TURN:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TURN, 0.7, 0)){
                        stateMachine = StateMachine.STRAFE;
                    }
                    break;
                case STRAFE:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_RUNG, 0.7, 0)){
                        stateMachine = StateMachine.EXTEND_ARM;
                    }
                    break;
                case EXTEND_ARM:
                    //lower arm
                    shoulderCommand = -1;
                    linearSlideCmd = 1;
                    clawClosed = true;
                    runtime.reset();
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;

            }

            robotController.update(shoulderCommand, linearSlideCmd, 0,
                    clawClosed, false,
                    false, false);
            if (stateMachine == StateMachine.AT_TARGET){
                RobotController.WheelPower wheelPower = new RobotController.WheelPower(0,0,0,0);
                robotController.assignWheelPowers(wheelPower);
            } else {
                //nav calculates the power to set to each motor in a mecanum or tank drive. Use nav.getMotorPower to find that value.
                RobotController.WheelPower wheelPower = new RobotController.WheelPower(nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_FRONT),
                        nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_FRONT),
                        nav.getMotorPower(DriveToPoint.DriveMotor.LEFT_BACK),
                        nav.getMotorPower(DriveToPoint.DriveMotor.RIGHT_BACK));
                robotController.assignWheelPowers(wheelPower);
            }

            telemetry.addData("current state:",stateMachine);

            telemetry.update();

        }
    }
}
