package org.firstinspires.ftc.teamcode.TEST.ServoTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "SERVO TEST : flap", group = "test")

public class flapServoTest extends LinearOpMode{

    private static Servo flapServo;

    private static double servoPos = 0.5, FO = 0.28, FC = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {

        flapServo = hardwareMap.servo.get("flapServo");

        servoPos = 0.5;

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

            flapServo.setPosition(servoPos);

            telemetry.addData("flapServo: ", flapServo.getPosition());
            telemetry.update();

        }
    }
}

