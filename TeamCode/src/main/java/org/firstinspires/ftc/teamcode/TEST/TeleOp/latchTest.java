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

@TeleOp(name = "latching test with variable power", group = "final")

public class latchTest extends LinearOpMode {

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

    private static double FO = 0.28, FC = 0.4;

    private double cPos;
    private double cMid;
    private double cInitial;
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
        cOpen = 0.92;
        cClose = 0.03;
        cMid = 0.6;
        cInitial = 0.01;


        //DROPPING
        dropperDC = hardwareMap.dcMotor.get("dropDC");
        dropperServo = hardwareMap.servo.get("dServo");
        dropperLimit = hardwareMap.get(ModernRoboticsTouchSensor.class, "D");

        dLoad = 0.71;
        dUnload = 0.3;

        dropperDC.setDirection(REVERSE);

        dropperDC.setMode(STOP_AND_RESET_ENCODER);
        dropperDC.setMode(RUN_USING_ENCODER);

        //DRIVE ENCODERS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        xUp = 0.4;
        yUp = 0.59;

        //INITIALIZATION
        dropperServo.setPosition(dLoad);
        collectorServo.setPosition(cInitial);
        xServo.setPosition(xUp);
        yServo.setPosition(yUp);
        flapServo.setPosition(FC);
        while (collectorDC.getCurrentPosition() < 240) {
            collectorDC.setPower(0.2);
        }
        collectorDC.setPower(0);
        collectorServo.setPosition(cClose);
        collectorDC.setMode(STOP_AND_RESET_ENCODER);
        collectorDC.setMode(RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

//            gamepad1.setJoystickDeadzone(0.3f); //THIS WAS NEVER ON

            /**DRIVING*/
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

            /**LATCHING*/
            if (!latchUpperLimit.isPressed()) {
                if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                    latchingDC.setPower(gamepad2.right_trigger);
                }
            }
            if (latchUpperLimit.isPressed()) {
                if (gamepad2.dpad_up && !gamepad2.dpad_down) {
                    latchingDC.setPower(0);
                }
            }
            if (!latchLowerLimit.isPressed()) {
                if (!gamepad2.dpad_up && gamepad2.dpad_down) {
                    latchingDC.setPower(-gamepad2.right_trigger);
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
        }
    }
}
//arm closed = 0.26