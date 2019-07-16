
package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Manual Test")
@Disabled

public class Manual_Test extends OpMode {

    public DcMotor motor1 = null;
    public DcMotor motor2 = null;
    public DcMotor motor3 = null;

    public Servo servo1 = null;
    public Servo servo2 = null;

    @Override
    public void init() {
        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");
       // motor3 = hardwareMap.dcMotor.get("motor3");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

    }

    @Override
    public void loop() {


        if (gamepad1.dpad_down) {
            servo1.setPosition(1);
        } else if (gamepad1.dpad_up) {
            servo1.setPosition(-1);
        } else {
            servo1.setPosition(0);
        }

        if (gamepad1.dpad_right) {
            servo2.setPosition(1);
        } else if (gamepad1.dpad_left) {
            servo2.setPosition(-1);
        } else {
            servo2.setPosition(0);
        }

        if (gamepad1.a) {
            motor1.setPower(1);
        } else if (gamepad1.b) {
            motor1.setPower(-1);
        } else {
            motor1.setPower(0);
        }

        if (gamepad1.x) {
            motor2.setPower(1);
        } else if (gamepad1.y) {
            motor2.setPower(-1);
        } else {
            motor2.setPower(0);

           // if (gamepad2.a) {
             //   motor3.setPower(.6);
          //  } else if (gamepad2.b) {
                //motor3.setPower(-.6);
          //  } else {
               // motor3.setPower(0);
           // }

           // if (gamepad2.x) {
             //   motor4.setPower(.6);
          //  } else if (gamepad2.y) {
            //    motor4.setPower(-.6);
          //  } else {
            //    motor4.setPower(0);
            }


        }

    }
