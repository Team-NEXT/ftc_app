package org.firstinspires.ftc.teamcode.TEST.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@TeleOp(name = "EncoderTest", group = "test")

public class Encoder extends LinearOpMode{

    //Gamepad 1

    private DcMotor yAxisDC;
    private DcMotor motorFrontRight;
//    private DcMotor motorBackRight;

    private static int x_pos;
    private static int y_pos;

    @Override
    public void runOpMode() throws InterruptedException {

        yAxisDC = hardwareMap.dcMotor.get("yAxisEncoder");
        motorFrontRight = hardwareMap.dcMotor.get("frontRight");

        yAxisDC.setMode(STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(STOP_AND_RESET_ENCODER);

        yAxisDC.setMode(RUN_USING_ENCODER);
        motorFrontRight.setMode(RUN_USING_ENCODER);

//        x_pos = 0;
        y_pos = 0;

        waitForStart();

        while (opModeIsActive()){

//            x_pos = motorBackRight.getCurrentPosition();
            y_pos = yAxisDC.getCurrentPosition();

//            if (gamepad1.x) {
//                motorBackRight.setMode(STOP_AND_RESET_ENCODER);
//                motorBackRight.setMode(RUN_USING_ENCODER);
//            }

            if (gamepad1.y) {
                yAxisDC.setMode(STOP_AND_RESET_ENCODER);
                yAxisDC.setMode(RUN_USING_ENCODER);
            }



//            telemetry.addData("x = ", x_pos);
            telemetry.addData("FR = ", motorFrontRight.getCurrentPosition());
            telemetry.addData("y = ", y_pos);
            telemetry.update();

        }
    }
}