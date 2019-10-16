package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//*** Created by Quick's on 10/14/2017.

//@Disabled
@TeleOp(name = "Testing2019")


public class Testing2019 extends OpMode{
    public LynxI2cColorRangeSensor color1 = null;

    public Servo servo1 = null;
    public Servo servo2 = null;


    @Override
    public void init() {

        servo1 = hardwareMap.servo.get ("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class,"color1");


    }

    @Override
    public void loop(){



    }
}
