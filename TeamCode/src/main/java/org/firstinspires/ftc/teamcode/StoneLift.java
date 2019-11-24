package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "StoneLift", group = "TeleOp")
//@Disabled
public class StoneLift extends LinearOpMode {

    DcMotor tilt_motor;
    DcMotor slide_motor;

    Servo servo;

    public void runOpMode() {

        servo = hardwareMap.servo.get("stone_servo");

        tilt_motor = hardwareMap.dcMotor.get("tilt_motor");
        slide_motor = hardwareMap.dcMotor.get("slide_motor");

        tilt_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        tilt_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        slide_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        tilt_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad2.dpad_up){
                tilt_motor.setPower(0.75);
            }else if(gamepad2.dpad_down){
                tilt_motor.setPower(-0.75);
            }else{
                tilt_motor.setPower(0);
            }

            if(gamepad2.y){
                slide_motor.setPower(0.75);
            }else if(gamepad2.a){
                slide_motor.setPower(-0.75);
            }else{
                slide_motor.setPower(0);
            }

            if(gamepad2.right_bumper){
                servo.setPosition(1);
            }else if(gamepad2.left_bumper){
                servo.setPosition(0);
            }

            telemetry.addData("Servo Position: ", servo.getPosition());
            telemetry.addData("Encoder Position (tilt_motor) : ", tilt_motor.getCurrentPosition());
            telemetry.addData("Encoder Position (slide_motor) : ", slide_motor.getCurrentPosition());
            telemetry.update();
        }
    }

}
