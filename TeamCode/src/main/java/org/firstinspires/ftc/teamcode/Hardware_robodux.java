package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.hardware.configuration.LynxI2cDeviceConfiguration;


public class Hardware_robodux {

    public DcMotor right_front_drive = null;
    public DcMotor right_rear_drive = null;
    public DcMotor left_front_drive = null;
    public DcMotor left_rear_drive = null;

    public DcMotor liftr = null;
    public DcMotor liftl = null;
    public DcMotor intaker = null;
    public DcMotor intakel = null;

    public Servo clampr = null;
    public Servo clampl = null;

    public Servo intake_sevr = null;
    public Servo intake_sevl = null;
    public Servo jewel_arm = null;

    public Servo right_jewel_stop = null;
    public Servo left_jewel_stop = null;


    HardwareMap map = null;

    public void init(HardwareMap aMap) {

        map = aMap;

        right_front_drive = map.dcMotor.get("right_front_drive");
        right_rear_drive = map.dcMotor.get("right_rear_drive");
        left_front_drive = map.dcMotor.get("left_front_drive");
        left_rear_drive = map.dcMotor.get("left_rear_drive");

        liftr = map.dcMotor.get("liftr");
        liftl = map.dcMotor.get("liftl");
        intaker = map.dcMotor.get("intaker");
        intakel = map.dcMotor.get("intakel");

        clampr = map.servo.get("clampr");
        clampl = map.servo.get("clampl");

        intake_sevl = map.servo.get("intake_servl");
        intake_sevr = map.servo.get("intake_servr");

        jewel_arm = map.servo.get("jewel");

        right_jewel_stop = map.servo.get("right_jewel_stop");
        left_jewel_stop = map.servo.get("left_jewel_stop");


        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        //right_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        //right_front_drive.setDirection(DcMotor.Direction.REVERSE);;
        //set direction on motors

        intakel.setDirection(DcMotorSimple.Direction.REVERSE);
        intaker.setDirection(DcMotorSimple.Direction.REVERSE);

        liftr.setDirection(DcMotorSimple.Direction.REVERSE);

        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stopRobot() {

        right_front_drive.setPower(0.0);
        right_rear_drive.setPower(0.0);
        left_front_drive.setPower(0.0);
        left_rear_drive.setPower(0.0);
    }


}
