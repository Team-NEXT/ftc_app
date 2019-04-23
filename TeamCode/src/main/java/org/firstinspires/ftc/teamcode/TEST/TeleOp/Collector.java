package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import android.view.Display;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "Collector", group = "mech-test")

public class Collector extends LinearOpMode {

    //DRIVE
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;

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

    //ENCODER SERVO
    private Servo xServo;
    private Servo yServo;

    @Override
    public void runOpMode() throws InterruptedException {

        //DRIVING
        motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        motorBackRight = hardwareMap.dcMotor.get("backRight");

        motorFrontRight.setDirection(REVERSE);
        motorBackRight.setDirection(REVERSE);

        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(RUN_USING_ENCODER);
        motorFrontLeft.setMode(RUN_USING_ENCODER);
        motorBackRight.setMode(RUN_USING_ENCODER);
        motorBackLeft.setMode(RUN_USING_ENCODER);

        //ENCODER SERVOS
        xServo = hardwareMap.servo.get("xServo");
        yServo = hardwareMap.servo.get("yServo");

        //COLLECTOR
        collectorDC = hardwareMap.dcMotor.get("collectDC");
        sweeperServo = hardwareMap.crservo.get("sweepServo");
//        sweeperServo.setDirection(REVERSE);
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
        cMid = 0.61;
        cDrop = 0.25;

        collectorServo.setPosition(cClose);
        xServo.setPosition(0.4);
        yServo.setPosition(0.59);

        waitForStart();

        while (opModeIsActive()) {

//            gamepad1.setJoystickDeadzone(0.3f); //THIS WAS NEVER ON

            if (gamepad1.left_stick_button) {
                collectorDC.setMode(STOP_AND_RESET_ENCODER);
                collectorDC.setMode(RUN_USING_ENCODER);
            }

            /**DRIVING*/
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            telemetry.addData("r = ", r);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            telemetry.addData("robotAngle = ", robotAngle);
            double rightX = gamepad1.right_stick_x;
            telemetry.addData("rightX = ", rightX);
            final double v1 = r * Math.cos(robotAngle) + rightX;
            telemetry.addData("front left power = ", v1);
            final double v2 = r * Math.sin(robotAngle) - rightX;
            telemetry.addData("front right power = ", v2);
            final double v3 = r * Math.sin(robotAngle) + rightX;
            telemetry.addData("back left power = ", v3);
            final double v4 = r * Math.cos(robotAngle) - rightX;
            telemetry.addData("back right power = ", v4);

            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);


            /**COLLECTION MECH*/

            if (collectorLimit.isPressed()) {
                if (gamepad1.left_trigger < 0.02 && gamepad1.right_trigger >= 0.02) {
                    collectorDC.setPower(0);
                }
            } else {
                if (gamepad1.left_trigger < 0.02 && gamepad1.right_trigger >= 0.02) {
                    collectorDC.setPower(-gamepad1.right_trigger);
                }
            }

            if (collectorDC.getCurrentPosition() > 2000) {
                if (gamepad1.left_trigger >= 0.02 && gamepad1.right_trigger < 0.02) {
                    collectorDC.setPower(0);
                }
            } else {
                if (gamepad1.left_trigger >= 0.02 && gamepad1.right_trigger < 0.02) {
                    collectorDC.setPower(gamepad1.left_trigger);
                }
            }

            if (gamepad1.left_trigger < 0.02 && gamepad1.right_trigger < 0.02) {
                collectorDC.setPower(0);
            }

            if (gamepad1.left_bumper) {
                flapServo.setPosition(FC);
                while (collectorServo.getPosition() < cOpen) {
                    cPos = collectorServo.getPosition();
                    cPos += 0.05;
                    collectorServo.setPosition(cPos);
                }
                if (cPos > cOpen) {
                    cPos = cOpen;
                    collectorServo.setPosition(cPos);
                }
                sweeperServo.setPower(-1);
            }

            if (gamepad1.right_bumper) {
                flapServo.setPosition(FC);

                sweeperServo.setPower(-1);

                while (collectorServo.getPosition() > cMid) {
                    cPos = collectorServo.getPosition();
                    cPos -= 0.05;
                    collectorServo.setPosition(cPos);
                }

                if (cPos < cMid) {
                    cPos = cMid;
                    collectorServo.setPosition(cPos);
                }

                sweeperServo.setPower(-0.6);

                while (collectorDC.getCurrentPosition() > 400) {
                    collectorDC.setPower(-1);
                }

                // sweeperServo.setPower(0);

                while (collectorDC.getCurrentPosition() <= 400 && !collectorLimit.isPressed()) {
                    collectorDC.setPower(-0.4);
                }

                collectorDC.setPower(0);

                collectorDC.setMode(STOP_AND_RESET_ENCODER);
                collectorDC.setMode(RUN_USING_ENCODER);

                while (collectorServo.getPosition() >= cClose) {
                    cPos = collectorServo.getPosition();
                    cPos -= 0.06;
                    collectorServo.setPosition(cPos);
                }

                if (cPos < cClose) {
                    cPos = cClose;
                    collectorServo.setPosition(cPos);
                }

                flapServo.setPosition(FO);
            }
            if (gamepad1.a && !gamepad1.b && !gamepad1.y) {
                sweeperServo.setPower(1);
            }
            if (!gamepad1.a && gamepad1.b && !gamepad1.y) {
                sweeperServo.setPower(0);
            }
            if (!gamepad1.a && !gamepad1.b && gamepad1.y) {
                sweeperServo.setPower(-1);
            }
            if (gamepad1.x) {
                if (collectorServo.getPosition() > 0.7) {
                    while (collectorServo.getPosition() > cMid) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cMid) {
                        collectorServo.setPosition(cMid);
                    }
                } else if (collectorServo.getPosition() > 0.15) {
                    while (collectorServo.getPosition() > cClose) {
                        cPos = collectorServo.getPosition();
                        cPos -= 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos < cClose) {
                        collectorServo.setPosition(cClose);
                    }
                } else {
                    while (collectorServo.getPosition() < cOpen) {
                        cPos = collectorServo.getPosition();
                        cPos += 0.05;
                        collectorServo.setPosition(cPos);
                    }
                    if (cPos > cOpen) {
                        collectorServo.setPosition(cOpen);
                    }
                }
//                    if (collectorServo.getPosition() < 0.2) {
//                        while (collectorServo.getPosition() < cMid) {
//                            cPos = collectorServo.getPosition();
//                            cPos += 0.05;
//                            collectorServo.setPosition(cPos);
//                        }
//                        if (cPos > cMid) {
//                            collectorServo.setPosition(cMid);
//                        }
//                    } else if (collectorServo.getPosition() < 0.75) {
//                        while (collectorServo.getPosition() < cOpen) {
//                            cPos = collectorServo.getPosition();
//                            cPos += 0.05;
//                            collectorServo.setPosition(cPos);
//                        }
//                        if (cPos > cOpen) {
//                            collectorServo.setPosition(cOpen);
//                        }
//                    } else {
//                        while (collectorServo.getPosition() > cClose) {
//                            cPos = collectorServo.getPosition();
//                            cPos -= 0.05;
//                            collectorServo.setPosition(cPos);
//                        }
//                        if (cPos < cClose) {
//                            collectorServo.setPosition(cClose);
//                        }
//                    }
            }
        }
    }
}

//    void COLLECTOREXTCLOSE (double power) {
//        sweeperDC.setPower(1);
//        while (collectorServo.getPosition() > cMid) {
//            cPos = collectorServo.getPosition();
//            cPos -= cSpeed;
//            collectorServo.setPosition(cPos);
//        }
//        if (cPos < cMid) {
//            cPos = cMid;
//            collectorServo.setPosition(cPos);
//        }
//        sweeperDC.setPower(0);
//        while (!collectorExtLimit.isPressed()) {
//            collectorDC.setPower(power);
//        }
//        collectorDC.setPower(0);
//    }

//arm closed = 0.26