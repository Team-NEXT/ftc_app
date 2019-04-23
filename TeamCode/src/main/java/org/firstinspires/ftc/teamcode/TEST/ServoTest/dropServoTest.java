package org.firstinspires.ftc.teamcode.TEST.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST : dropper", group = "servo-test")

public class dropServoTest extends LinearOpMode{

    private static Servo dropServo;

    private static double servoPos = 0, dU = 0.27, dL = 0.71;

    @Override
    public void runOpMode() throws InterruptedException {

        dropServo = hardwareMap.servo.get("dServo");

        servoPos = 0;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
//                Thread.sleep(750);
//                if (!gamepad1.x) {
//                    xPos += 0.01;
//                }
                servoPos += 0.01;
            }

            if (gamepad1.a) {
//                 Thread.sleep(750);
//                if (!gamepad1.a) {
//                    xPos -= 0.01;
//                }
                servoPos -= 0.01;
            }

            if (gamepad1.left_bumper) {
                servoPos = dL;
            }

            if (gamepad1.right_bumper) {
                servoPos = dU;
            }

            dropServo.setPosition(servoPos);

            telemetry.addData("dServo: ", dropServo.getPosition());
            telemetry.update();

        }
    }
}

