package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.control.ArmMode;
import org.firstinspires.ftc.teamcode.control.RobotController;

@Autonomous(group="OdomAutonomous", preselectTeleOp = "Main_Teleop")
//@Disabled

public class OdomAutoDeliver3Specimens extends LinearOpMode {
    RobotController robotController = new RobotController();
    private final ElapsedTime runtime = new ElapsedTime();

    DriveToPoint nav = new DriveToPoint(this); //OpMode member for the point-to-point navigation class

    enum StateMachine {
        WAITING_FOR_START,
        AT_TARGET,
        GO_TO_OBSERVATION,
        RAISE_ARM_HIGH_SPECIMEN,
        DRIVE_TO_HIGH_SPECIMEN,
        DELIVER_SPECIMEN,
        OPEN_CLAW,
        PRE_MOVE_TO_SPEC0,
        MOVE_TO_SPEC0,
        CLOSE_CLAW_SPEC0,
        LIFT_ARM_SPEC0,
        GO_TO_PRE_DELIVER_POS,
        GO_TO_DELIVER_POS,
        DRIVE_TO_HIGH_SPECIMEN0,
        DELIVER_SPECIMEN0,
        OPEN_CLAW0,
        DRIVE_TO_SPEC1_STEP1,
        DRIVE_TO_SPEC1_STEP2,
        DRIVE_TO_SPEC1_STEP3,
        DRIVE_TO_SPEC1_STEP4,
        PRE_MOVE_TO_SPECA,
        MOVE_TO_SPECA,
        CLOSE_CLAW_SPECA,
        LIFT_ARM_SPECA,
        GO_TO_PRE_DELIVER_POSA,
        GO_TO_DELIVER_POSA,
        DRIVE_TO_HIGH_SPECIMENA,
        DELIVER_SPECIMENA,
        OPEN_CLAWA,
        DRIVE_TO_SPEC2_STEP1,
        DRIVE_TO_SPEC2_STEP2,
        DRIVE_TO_SPEC2_STEP3,
        DRIVE_TO_OBSERVATION_ZONE,
//        DRIVE_TO_SPEC3_STEP1,
//        DRIVE_TO_SPEC3_STEP2,
//        DRIVE_TO_SPEC3_STEP3
    }

    static final Pose2D TARGET_HIGH_SPECIMEN = new Pose2D(DistanceUnit.MM,750,0,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_MOVE_SPEC0 = new Pose2D(DistanceUnit.MM,300, -1000, AngleUnit.DEGREES, 175);
    static final Pose2D TARGET_GRAB_SPEC0 = new Pose2D(DistanceUnit.MM,166, -1000, AngleUnit.DEGREES, 175);
    static final Pose2D TARGET_PRE_DELIVER0 = new Pose2D(DistanceUnit.MM,400, -30, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_HIGH_SPECIMEN0 = new Pose2D(DistanceUnit.MM,770,-30,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_SPEC1_STEP1 = new Pose2D(DistanceUnit.MM,400, -950, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC1_STEP2 = new Pose2D(DistanceUnit.MM,1300, -950, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC1_STEP3 = new Pose2D(DistanceUnit.MM,1300, -1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC1_STEP4 = new Pose2D(DistanceUnit.MM,200, -1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_MOVE_SPECA = new Pose2D(DistanceUnit.MM,300, -1000, AngleUnit.DEGREES, 175);
    static final Pose2D TARGET_GRAB_SPECA = new Pose2D(DistanceUnit.MM,166, -1000, AngleUnit.DEGREES, 175);
    static final Pose2D TARGET_PRE_DELIVERA = new Pose2D(DistanceUnit.MM,400, -60, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_HIGH_SPECIMENA = new Pose2D(DistanceUnit.MM,780,-60,AngleUnit.DEGREES,0);
    static final Pose2D TARGET_SPEC2_STEP1 = new Pose2D(DistanceUnit.MM,1300, -1200, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC2_STEP2 = new Pose2D(DistanceUnit.MM,1300, -1300, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC2_STEP3 = new Pose2D(DistanceUnit.MM,350, -1300, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC3_STEP1 = new Pose2D(DistanceUnit.MM,1300, -1300, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC3_STEP2 = new Pose2D(DistanceUnit.MM,1300, -1550, AngleUnit.DEGREES, 0);
    static final Pose2D TARGET_SPEC3_STEP3 = new Pose2D(DistanceUnit.MM,350, -1550, AngleUnit.DEGREES, 0);

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
                    stateMachine = StateMachine.RAISE_ARM_HIGH_SPECIMEN;
                    runtime.reset();
                    break;
                case RAISE_ARM_HIGH_SPECIMEN:
                    robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.DRIVE_TO_HIGH_SPECIMEN;
                    }
                    break;
                case DRIVE_TO_HIGH_SPECIMEN:
                    /*
                    drive the robot to the first target, the nav.driveTo function will return true once
                    the robot has reached the target, and has been there for (holdTime) seconds.
                    Once driveTo returns true, it prints a telemetry line and moves the state machine forward.
                     */
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_HIGH_SPECIMEN, 0.45, 0)){
                        stateMachine = StateMachine.DELIVER_SPECIMEN;
                        runtime.reset();
                    }
                    break;
                case DELIVER_SPECIMEN:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    shoulderCommand = -1;
                    clawClosed = true;
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.OPEN_CLAW;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAW:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.25) {
                        stateMachine = StateMachine.DRIVE_TO_SPEC1_STEP1;
                        runtime.reset();
                    }
                    break;
                case DRIVE_TO_SPEC1_STEP1:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = false;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP1, 0.8, 0)){
                        stateMachine = StateMachine.DRIVE_TO_SPEC1_STEP2;
                    }
                    break;
                case DRIVE_TO_SPEC1_STEP2:
                    //lower arm
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP2, 0.8, 0)){
                        stateMachine = StateMachine.DRIVE_TO_SPEC1_STEP3;
                    }
                    break;
                case DRIVE_TO_SPEC1_STEP3:
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP3, 0.8, 0)){
                        stateMachine = StateMachine.DRIVE_TO_SPEC1_STEP4;
                    }
                    break;
                case DRIVE_TO_SPEC1_STEP4:
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC1_STEP4, 0.8, 0)){
                        stateMachine = StateMachine.PRE_MOVE_TO_SPEC0;
                    }
                    break;
                case PRE_MOVE_TO_SPEC0:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_MOVE_SPEC0, 0.8, 0)){
                        stateMachine = StateMachine.MOVE_TO_SPEC0;
                        runtime.reset();
                    }
                    break;
                case MOVE_TO_SPEC0:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_GRAB_SPEC0, 0.45, 0.25)){
                        stateMachine = StateMachine.CLOSE_CLAW_SPEC0;
                        runtime.reset();
                    }
                    break;
                case CLOSE_CLAW_SPEC0:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = true;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.LIFT_ARM_SPEC0;
                        runtime.reset();
                    }
                    break;
                case LIFT_ARM_SPEC0:
                    robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.GO_TO_PRE_DELIVER_POS;
                    }
                    break;
                case GO_TO_PRE_DELIVER_POS:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_PRE_DELIVER0, 0.8, 0)){
                        stateMachine = StateMachine.DRIVE_TO_HIGH_SPECIMEN0;
                    }
                    break;
                case DRIVE_TO_HIGH_SPECIMEN0:
                    //raisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisin
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_HIGH_SPECIMEN0, 0.45, 0)){
                        stateMachine = StateMachine.DELIVER_SPECIMEN0;
                        runtime.reset();
                    }
                    break;
                case DELIVER_SPECIMEN0:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    shoulderCommand = -1;
                    clawClosed = true;
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.OPEN_CLAW0;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAW0:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.PRE_MOVE_TO_SPECA;
                        runtime.reset();
                    }
                    break;

                case PRE_MOVE_TO_SPECA:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_MOVE_SPECA, 0.8, 0)){
                        stateMachine = StateMachine.MOVE_TO_SPECA;
                        runtime.reset();
                    }
                    break;
                case MOVE_TO_SPECA:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.setArmMode(ArmMode.GRAB_SPECIMEN);
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_GRAB_SPECA, 0.45, 0.25)){
                        stateMachine = StateMachine.CLOSE_CLAW_SPECA;
                        runtime.reset();
                    }
                    break;
                case CLOSE_CLAW_SPECA:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = true;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.LIFT_ARM_SPECA;
                        runtime.reset();
                    }
                    break;
                case LIFT_ARM_SPECA:
                    robotController.setArmMode(ArmMode.HIGH_SPECIMEN);
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (runtime.seconds() >= 1.0){
                        stateMachine = StateMachine.GO_TO_PRE_DELIVER_POSA;
                    }
                    break;
                case GO_TO_PRE_DELIVER_POSA:
                    //raise the arm
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_PRE_DELIVERA, 0.8, 0)){
                        stateMachine = StateMachine.DRIVE_TO_HIGH_SPECIMENA;
                    }
                    break;
                case DRIVE_TO_HIGH_SPECIMENA:
                    //raisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisinraisin
                    shoulderCommand = 0;
                    clawClosed = true;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_HIGH_SPECIMENA, 0.45, 0)){
                        stateMachine = StateMachine.DELIVER_SPECIMENA;
                        runtime.reset();
                    }
                    break;
                case DELIVER_SPECIMENA:
                    robotController.setArmMode(ArmMode.DRIVER_CONTROL);
                    shoulderCommand = -1;
                    clawClosed = true;
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.OPEN_CLAWA;
                        runtime.reset();
                    }
                    break;
                case OPEN_CLAWA:
                    //make the claw let go of the specimen
                    shoulderCommand = 0;
                    clawClosed = false;
                    robotController.updateDriveCommands(0, 0, 0, false);
                    if (runtime.seconds() >= 0.5) {
                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP3;
                        runtime.reset();
                    }
                    break;
                case DRIVE_TO_SPEC2_STEP1:
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP1, 0.7, 0)){
                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP2;
                    }
                    break;
                case DRIVE_TO_SPEC2_STEP2:
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP2, 0.7, 0)){
                        stateMachine = StateMachine.DRIVE_TO_SPEC2_STEP3;
                    }
                    break;
                case DRIVE_TO_SPEC2_STEP3:
                    shoulderCommand = -1;
                    linearSlideCmd = -1;
                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC2_STEP3, 0.8, 0)){
                        stateMachine = StateMachine.AT_TARGET;
                    }
                    break;
//                case DRIVE_TO_SPEC3_STEP1:
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP1, 0.7, 0)){
//                        stateMachine = StateMachine.DRIVE_TO_SPEC3_STEP2;
//                    }
//                    break;
//                case DRIVE_TO_SPEC3_STEP2:
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP2, 0.7, 0)){
//                        stateMachine = StateMachine.DRIVE_TO_SPEC3_STEP3;
//                    }
//                    break;
//                case DRIVE_TO_SPEC3_STEP3:
//                    shoulderCommand = 0;
//                    linearSlideCmd = 0;
//                    clawClosed = true;
//                    if (nav.driveTo(robotController.getOdometryPosition(), TARGET_SPEC3_STEP3, 0.7, 0)){
//                        stateMachine = StateMachine.AT_TARGET;
//                    }
//                    break;
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
