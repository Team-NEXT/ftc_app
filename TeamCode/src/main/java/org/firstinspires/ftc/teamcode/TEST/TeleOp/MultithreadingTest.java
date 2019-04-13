package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "mttest", group = "final")

public class MultithreadingTest extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

    //LATCHING
    private DcMotor latchingDC;
    private ModernRoboticsTouchSensor latchUpperLimit;
    private ModernRoboticsTouchSensor latchLowerLimit;

    //COLLECTOR
    private DcMotor collectorDC;
    private CRServo sweeperServo;
    private Servo collectorServo;
    private Servo flapServo;
    private ModernRoboticsTouchSensor collectorLimit;

    private static double FO = 0.55, FC = 0.18;

    private double cPos;
    private double cMid;
    private double cDrop;
    private double cOpen;
    private double cClose;

//    private double cSpeed;
//    private double cExtPos;

    //DROPPING
    private DcMotor dropperDC;
    private Servo dropperServo;
    private ModernRoboticsTouchSensor dropperLimit;

    private double dLoad;
    private double dUnload;
    private double dPos;

    //DRIVE ENCODERS
    private Servo xServo;
    private Servo yServo;

    private double xUp;
    private double yUp;

    private double startTime;

    public boolean dExit = false, cExit = false;

    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVING
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontLeft.setDirection(REVERSE);
        motorBackLeft.setDirection(REVERSE);

//        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
//        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);
//
//        motorFrontRight.setMode(RUN_USING_ENCODER);
//        motorFrontLeft.setMode(RUN_USING_ENCODER);
//        motorBackRight.setMode(RUN_USING_ENCODER);
//        motorBackLeft.setMode(RUN_USING_ENCODER);

        //LATCHING
        latchingDC = hardwareMap.dcMotor.get("latchingDC");
        latchingDC.setDirection(REVERSE);
        latchUpperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "LD");
        latchLowerLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "LU");

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperServo = hardwareMap.crservo.get("sweepServo");
        sweeperServo.setDirection(REVERSE);
        flapServo = hardwareMap.servo.get("flapServo");
        collectorServo = hardwareMap.servo.get("cServo");
        collectorLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "C");

        //sweeperDC.setDirection(REVERSE);
        //collectorDC.setDirection(REVERSE);

        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

//        cSpeed = 0.07;

//        cExtPos = collectorDC.getCurrentPosition();
        cOpen = 0.9;
        cClose = 0.05;
        cMid = 0.6;
        cDrop = 0.25;

        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "D");

        dLoad = 0.71;
        dUnload = 0.27;

        dropperDC.setDirection(REVERSE);

        dropperDC.setMode(STOP_AND_RESET_ENCODER);
        dropperDC.setMode(RUN_USING_ENCODER);

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.4;
        yUp = 0.59;

        //INITIALIZATION
        xServo.setPosition(xUp);
        yServo.setPosition(yUp);
        flapServo.setPosition(FC);

        Thread driveThread = new DriveThread();
        Thread collectThread = new CollectThread();
        Thread dropThread = new DropThread();
        Thread latchThread = new LatchThread();

        waitForStart();

        driveThread.start();
        collectThread.start();
        dropThread.start();
        latchThread.start();

        while (opModeIsActive()) {

            telemetry.addData("run time: ", this.getRuntime());
            telemetry.update();

        }

        driveThread.interrupt();
        collectThread.interrupt();
        dropThread.interrupt();
        latchThread.interrupt();

    }

    private class DriveThread extends Thread {

        public DriveThread() {

            this.setName("DriveThread");

        }

        @Override
        public void run() {

            while (!isInterrupted()) {

//                gamepad1.setJoystickDeadzone(0.3f); //THIS WAS NEVER ON

                /**DRIVING*/
                if (gamepad1.right_trigger < 0.1 && gamepad1.left_trigger < 0.2) {

                    telemetry.addLine("full speed");

                    double hypot = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                    telemetry.addData("r = ", hypot);

                    double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
                    telemetry.addData("robotAngle = ", robotAngle);

                    double rightStickModifier = -gamepad1.right_stick_x;
                    telemetry.addData("rightX = ", rightStickModifier);

                    final double v1 = hypot * Math.cos(robotAngle) + rightStickModifier;
                    telemetry.addData("front left power = ", motorFrontLeft.getPower());

                    final double v2 = hypot * Math.sin(robotAngle) - rightStickModifier;
                    telemetry.addData("front right power = ", motorFrontRight.getPower());

                    final double v3 = hypot * Math.sin(robotAngle) + rightStickModifier;
                    telemetry.addData("back left power = ", motorBackLeft.getPower());

                    final double v4 = hypot * Math.cos(robotAngle) - rightStickModifier;
                    telemetry.addData("back right power = ", motorBackRight.getPower());

                    telemetry.update();

                    motorFrontRight.setPower(v2);
                    motorFrontLeft.setPower(v1);
                    motorBackLeft.setPower(v3);
                    motorBackRight.setPower(v4);

                }

                /**INCHING*/
                if (gamepad1.right_trigger > 0.1 && !(gamepad1.left_trigger > 0.1)) {

                    telemetry.addLine("INCHING");
                    telemetry.update();

                    if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                        motorFrontRight.setPower(0);
                        motorFrontLeft.setPower(0);
                        motorBackLeft.setPower(0);
                        motorBackRight.setPower(0);
                    }

                    if (gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                        motorFrontRight.setPower(0.2);
                        motorFrontLeft.setPower(0.2);
                        motorBackLeft.setPower(0.2);
                        motorBackRight.setPower(0.2);
                    }

                    if (!gamepad1.dpad_up && gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                        motorFrontRight.setPower(-0.2);
                        motorFrontLeft.setPower(-0.2);
                        motorBackLeft.setPower(-0.2);
                        motorBackRight.setPower(-0.2);
                    }

                    if (!gamepad1.dpad_up && !gamepad1.dpad_down && gamepad1.dpad_left && !gamepad1.dpad_right) {
                        motorFrontRight.setPower(0.2);
                        motorFrontLeft.setPower(-0.2);
                        motorBackLeft.setPower(-0.2);
                        motorBackRight.setPower(0.2);
                    }

                    if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && gamepad1.dpad_right) {
                        motorFrontRight.setPower(-0.2);
                        motorFrontLeft.setPower(0.2);
                        motorBackLeft.setPower(0.2);
                        motorBackRight.setPower(-0.2);
                    }

                }

                /**MEDIUM*/
                if (gamepad1.left_trigger > 0.1 &&!(gamepad1.right_trigger > 0.1)) {
                    telemetry.addLine("full speed");

                    double hypot = Math.hypot((gamepad1.left_stick_x*0.5), (gamepad1.left_stick_y*0.5));
                    telemetry.addData("r = ", hypot);

                    double robotAngle = Math.atan2(-(gamepad1.left_stick_y*0.5), (gamepad1.left_stick_x*0.5)) - Math.PI / 4;
                    telemetry.addData("robotAngle = ", robotAngle);

                    double rightStickModifier = -(gamepad1.right_stick_x*0.5);
                    telemetry.addData("rightX = ", rightStickModifier);

                    final double v1 = hypot * Math.cos(robotAngle) + rightStickModifier;
                    telemetry.addData("front left power = ", motorFrontLeft.getPower());

                    final double v2 = hypot * Math.sin(robotAngle) - rightStickModifier;
                    telemetry.addData("front right power = ", motorFrontRight.getPower());

                    final double v3 = hypot * Math.sin(robotAngle) + rightStickModifier;
                    telemetry.addData("back left power = ", motorBackLeft.getPower());

                    final double v4 = hypot * Math.cos(robotAngle) - rightStickModifier;
                    telemetry.addData("back right power = ", motorBackRight.getPower());

                    telemetry.update();

                    motorFrontRight.setPower(v2);
                    motorFrontLeft.setPower(v1);
                    motorBackLeft.setPower(v3);
                    motorBackRight.setPower(v4);
                }

            }

        }

    }

    private class CollectThread extends Thread {

        public CollectThread() {

            this.setName("CollectThread");

        }

        @Override
        public void run() {

            while (!isInterrupted()) {

                /**SWEEPER*/
                if (gamepad2.right_bumper && !gamepad2.left_bumper && gamepad2.left_trigger < 0.1) {
                    sweeperServo.setPower(1);
                }
                if (gamepad2.left_trigger > 0.1 && !gamepad2.right_bumper && !gamepad1.left_bumper  && !gamepad2.b) {
                    sweeperServo.setPower(-gamepad2.left_trigger);
                }
                if (gamepad2.left_bumper && !gamepad2.right_bumper && gamepad2.left_trigger < 0.1  && !gamepad2.b) {
                    sweeperServo.setPower(0);
                }

                /**COLLECTOR*/
                if (gamepad2.y && !gamepad2.x && !gamepad2.b && (gamepad2.right_trigger < 0.1)) {
//                startTime = System.currentTimeMillis();
                    flapServo.setPosition(FC);
                    while (collectorServo.getPosition() < cOpen && !gamepad2.b) {
                        cPos = collectorServo.getPosition();
                        cPos += 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos > cOpen) {
                        cPos = cOpen;
                        collectorServo.setPosition(cPos);
                    }
                }
//            if (!collectorServoLimit.isPressed()) {
                if (!gamepad2.y && gamepad2.x && !gamepad2.b && (gamepad2.right_trigger < 0.1)) {
//                    startTime = System.currentTimeMillis();
                    flapServo.setPosition(FO);
                    while (collectorServo.getPosition() > cClose && !gamepad2.b) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cClose) {
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }
                }
//            }
                if (!gamepad2.y && !gamepad2.x && !gamepad2.b && (gamepad2.right_trigger > 0.1)) {
//                startTime = System.currentTimeMillis();
                    while (collectorServo.getPosition() < cMid && !gamepad2.b) {
                        cPos = collectorServo.getPosition();
                        cPos += 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos > cMid) {
                        cPos = cMid;
                        collectorServo.setPosition(cMid);
                    }
                }
                if (collectorDC.getCurrentPosition() < 1800) {
                    if (!gamepad2.a && gamepad2.b && (gamepad2.left_stick_x > -0.2)) {
                        collectorDC.setPower(1);
                    }
                    if (!gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x < -0.2)) {
                        collectorDC.setPower(0.4);
                    }
                }
                if (collectorDC.getCurrentPosition() >= 1800) {
                    if (!gamepad2.a && gamepad2.b && (gamepad2.left_stick_x > -0.2)) {
                        collectorDC.setPower(0);
                    }
                    if (!gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x < -0.2)) {
                        collectorDC.setPower(0);
                    }
                }
                if (!collectorLimit.isPressed()) {
                    if (gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x < 0.2)){
                        collectorDC.setPower(-1);
                    }
                    if (!gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x > 0.2)) {
                        collectorDC.setPower(-0.4);
                    }
                }
                if (collectorLimit.isPressed()) {
                    if (gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x < 0.2)) {
                        collectorDC.setPower(0);
                    }
                    if (!gamepad2.a && !gamepad2.b && (gamepad2.left_stick_x > 0.2)) {
                        collectorDC.setPower(0);
                    }
                }

                if (!gamepad2.b && !gamepad2.a && (gamepad2.left_stick_x<0.2) && (gamepad2.left_stick_x>-0.2)) {
                    collectorDC.setPower(0);
                }


                if (gamepad2.right_stick_button) {
//                COLLECTOREXTCLOSE(0.8);
                    sweeperServo.setPower(1);

                    cExit = false;

                    while (collectorServo.getPosition() > cMid && !cExit) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }

                    if (cPos < cMid) {
                        cPos = cMid;
                        collectorServo.setPosition(cPos);
                    }

                    sweeperServo.setPower(0.6);

                    while (collectorDC.getCurrentPosition() > 400 && !cExit) {
                        collectorDC.setPower(-1);
                    }

                   // sweeperServo.setPower(0);

                    while (collectorDC.getCurrentPosition() <= 400 && !collectorLimit.isPressed() && !cExit) {
                        collectorDC.setPower(-0.4);
                    }

                    collectorDC.setPower(0);

                    collectorDC.setMode(STOP_AND_RESET_ENCODER);
                    collectorDC.setMode(RUN_USING_ENCODER);

                    while (collectorServo.getPosition() >= cClose && !cExit) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }

                    if (cPos < cClose) {
                        cPos = cClose;
                        collectorServo.setPosition(cPos);
                    }

                    flapServo.setPosition(FO);
                }

            }

        }

    }

    private class DropThread extends Thread {

        public DropThread() {

            this.setName("DropThread");

        }

        @Override
        public void run() {

            while (!isInterrupted()) {

                /**DROPPER*/
                if (dropperLimit.isPressed()) {
                    dropperDC.setMode(STOP_AND_RESET_ENCODER);
                    dropperDC.setMode(RUN_USING_ENCODER);
                }

                if (!gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.a && gamepad1.b) {
//                startTime = System.currentTimeMillis();
                    while (dropperServo.getPosition() > dUnload) {
                        dPos = dropperServo.getPosition();
                        dPos -= 0.038;
                        dropperServo.setPosition(dPos);
                    }
                    if (dPos < dUnload) {
                        dPos = dUnload;
                        dropperServo.setPosition(dPos);
                    }
                }
                if (!gamepad1.b && gamepad1.a) {
                    // startTime = System.currentTimeMillis();
                    while (dropperServo.getPosition() < dLoad) {
                        dPos = dropperServo.getPosition();
                        dPos += 0.042;
                        dropperServo.setPosition(dPos);
                    }
                    if (dPos > dLoad) {
                        dPos = dLoad;
                        dropperServo.setPosition(dPos);
                    }
                }

                if (gamepad1. right_stick_button && !gamepad1.left_stick_button) {
                    dExit = false;
                    while (dropperServo.getPosition() > dLoad && !dExit) {
                        dPos = dropperServo.getPosition();
                        dPos += 0.04;
                        dropperServo.setPosition(dPos);
                    }
                    if (dPos < dLoad) {
                        dPos = dLoad;
                        dropperServo.setPosition(dPos);
                    }
//        startTime = System.currentTimeMillis();
                    while (dropperDC.getCurrentPosition() > 500 && !dExit) {
                        dropperDC.setPower(-1);
                    }
                    dropperDC.setPower(0);
                    while (!dropperLimit.isPressed() && !dExit) {
                        dropperDC.setPower(-0.3);
                    }
                    dropperDC.setMode(STOP_AND_RESET_ENCODER);
                    dropperDC.setMode(RUN_USING_ENCODER);

                    dropperDC.setPower(0);
                }

                if (!gamepad1. right_stick_button && gamepad1.left_stick_button && dropperDC.getCurrentPosition()<150) {
                    dExit = false;
                    dropperDC.setMode(STOP_AND_RESET_ENCODER);
                    dropperDC.setMode(RUN_USING_ENCODER);
                    while (collectorServo.getPosition() > cDrop && !dExit) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.07;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cDrop) {
                        collectorServo.setPosition(cDrop);
                    }
                    while (dropperDC.getCurrentPosition() < 1000 && !dExit) {
                        dropperDC.setPower(1);
                    }
                    dropperDC.setPower(0);
                    dropperDC.setPower(0);
                    dropperDC.setPower(0);
                    while (dropperServo.getPosition() > dUnload && !dExit) {
                        dPos = dropperServo.getPosition();
                        dPos -= 0.03;
                        dropperServo.setPosition(dPos);
                    }
                    if (dPos < dUnload) {
                        dropperServo.setPosition(dUnload);
                    }
                }

            if (dropperDC.getCurrentPosition() < 1000) {
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    dropperDC.setPower(1);
                }
            }
            if (dropperDC.getCurrentPosition() >= 1000) {
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    dropperDC.setPower(0);
                }
            }
                if (dropperLimit.isPressed()) {
                    if (!gamepad1.right_bumper && gamepad1.left_bumper) {
                        dropperDC.setPower(0);
                    }
                }
                if (!dropperLimit.isPressed()) {
                    if (dropperDC.getCurrentPosition() > 300) {
                        if (!gamepad1.right_bumper && gamepad1.left_bumper) {
                            dropperDC.setPower(-1);
                        }
                    }
                    if (dropperDC.getCurrentPosition() <= 300) {
                        if (!gamepad1.right_bumper && gamepad1.left_bumper) {
                            dropperDC.setPower(-0.4);
                        }
                    }
                }
                if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    dropperDC.setPower(0);
                }

                telemetry.addData("ddc: ", dropperDC.getCurrentPosition());
                telemetry.update();

            }

        }

    }

    private class LatchThread extends Thread {


        public  LatchThread() {

            this.setName("LatchThread");

        }

        @Override
        public void run() {

            while (!isInterrupted()) {

                /**LATCHING*/
                if (!latchUpperLimit.isPressed()) {
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        latchingDC.setPower(1);
                    }
                }
                if (latchUpperLimit.isPressed()) {
                    if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                        latchingDC.setPower(0);
                    }
                }
                if (!latchLowerLimit.isPressed()) {
                    if (!gamepad2.dpad_up && gamepad2.dpad_down) {
                        latchingDC.setPower(-1);
                    }
                }
                if (latchLowerLimit.isPressed()) {
                    if (!gamepad2.dpad_up && gamepad2.dpad_down) {
                        latchingDC.setPower(0);
                    }
                }
                if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                    latchingDC.setPower(0);
                }

                /**cExit && dExit Monitoring*/
                if (gamepad1.y) {
                    dExit = true;
                }
                if (gamepad2.left_stick_button) {
                    cExit = true;
                }

            }

        }

    }

}


//arm closed = 0.26
