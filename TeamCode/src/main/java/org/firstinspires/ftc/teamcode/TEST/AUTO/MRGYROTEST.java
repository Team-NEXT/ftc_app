/**2400 ticks = 1 encoder rotation*/

package org.firstinspires.ftc.teamcode.TEST.AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Autonomous(name = "MR GYRO TEST", group = "test")

public class MRGYROTEST extends LinearOpMode{

    private static DcMotor motorFrontRight;
    private static DcMotor motorBackRight;
    private static DcMotor motorFrontLeft;
    private static DcMotor motorBackLeft;

    private static double x_pos;
    private static double y_pos;

    private static Servo xServo;
    private static Servo yServo;

    private static double x_down = 0.72;
    private static double y_down = 0.08;

    // private static double x_encoder;
    // private static double y_encoder;

    private static double rampDownFactor = 0;
    private static double rampTicks = 0;

    private static double targetTicks = 0;
    private static final double finalTicks = 2400;
    private static final double singleTicks = 16;

    private static final double wheelDiameter = 48;


    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVE
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");

        motorFrontLeft.setDirection(REVERSE);
        motorBackLeft.setDirection(REVERSE);

        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);

        //ENCODER SERVOS
        xServo.setPosition(x_down);
        yServo.setPosition(y_down);

        // x_encoder = motorBackRight.getCurrentPosition();
        // y_encoder = motorFrontRight.getCurrentPosition();


        waitForStart();

        /**AUTO CODE*/

        FORWARD(1000, 0.8);

        Thread.sleep(1500);

        FORWARD(300, 0.8);
    }

    public static void FORWARD (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((targetTicks-50) > Math.abs(motorFrontRight.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void BACKWARD (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((targetTicks-50) > Math.abs(motorFrontRight.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void SWAYLEFT (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((targetTicks-50) > Math.abs(motorBackRight.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void SWAYRIGHT (int mmDistance, double power) {

        targetTicks = mmDistance * singleTicks;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while ((targetTicks-50) > Math.abs(motorBackRight.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void RAMP_FORWARD (int mmDistance, double power, double lowerLimit, double rampDownFactor) {

        targetTicks = mmDistance * singleTicks;

        rampTicks = targetTicks * 0.25;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (targetTicks*0.75 > Math.abs(motorFrontRight.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
        }

        while (targetTicks > Math.abs(motorFrontRight.getCurrentPosition())) {
            power -= rampDownFactor;
            if (power <= lowerLimit) {
                power = lowerLimit;
            }
            motorFrontRight.setPower(power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void RAMP_BACKWARD (int mmDistance, double power, double lowerLimit, double rampDownFactor) {

        targetTicks = mmDistance * singleTicks;

        rampTicks = targetTicks * 0.25;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (targetTicks*0.75 > Math.abs(motorFrontRight.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
        }

        while (targetTicks > Math.abs(motorFrontRight.getCurrentPosition())) {
            power -= rampDownFactor;
            if (power <= lowerLimit) {
                power = lowerLimit;
            }
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void RAMP_SWAYLEFT (int mmDistance, double power, double lowerLimit, double rampDownFactor) {

        targetTicks = mmDistance * singleTicks;

        rampTicks = targetTicks * 0.25;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (targetTicks*0.75 > Math.abs(motorBackRight.getCurrentPosition())) {
            motorFrontRight.setPower(power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(power);
        }

        while (targetTicks > Math.abs(motorBackRight.getCurrentPosition())) {
            power -= rampDownFactor;
            if (power <= lowerLimit) {
                power = lowerLimit;
            }
            motorFrontRight.setPower(power);
            motorBackRight.setPower(-power);
            motorFrontLeft.setPower(-power);
            motorBackLeft.setPower(power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

    public static void RAMP_SWAYRIGHT (int mmDistance, double power, double lowerLimit, double rampDownFactor) {

        targetTicks = mmDistance * singleTicks;

        rampTicks = targetTicks * 0.25;

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (targetTicks*0.75 > Math.abs(motorBackRight.getCurrentPosition())) {
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(-power);
        }

        while (targetTicks > Math.abs(motorBackRight.getCurrentPosition())) {
            power -= rampDownFactor;
            if (power <= lowerLimit) {
                power = lowerLimit;
            }
            motorFrontRight.setPower(-power);
            motorBackRight.setPower(power);
            motorFrontLeft.setPower(power);
            motorBackLeft.setPower(-power);
        }

        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);

    }

}