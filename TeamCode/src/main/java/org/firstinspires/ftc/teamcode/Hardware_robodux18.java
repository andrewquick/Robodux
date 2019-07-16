package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Hardware_robodux18 {

    public DcMotor right_front_drive = null;
    public DcMotor right_rear_drive = null;
    public DcMotor left_front_drive = null;
    public DcMotor left_rear_drive = null;

    public DcMotor lift = null;
    public DcMotor intake = null;
    public DcMotor intakearm = null;

    public Servo trapdoor = null;
    public Servo latch = null;

    public Servo dumpright = null;
    public Servo dumpleft = null;

    HardwareMap map = null;

    public void init(HardwareMap aMap) {

        map = aMap;

        right_front_drive = map.dcMotor.get("right_front_drive");
        right_rear_drive = map.dcMotor.get("right_rear_drive");
        left_front_drive = map.dcMotor.get("left_front_drive");
        left_rear_drive = map.dcMotor.get("left_rear_drive");

        lift = map.dcMotor.get("lift");
        intake = map.dcMotor.get("intake");
        intakearm = map.dcMotor.get("intakearm");

        trapdoor = map.servo.get("trapdoor");
        latch = map.servo.get("latch");

        dumpright = map.servo.get("dumpright");
        dumpleft = map.servo.get("dumpleft");

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        //right_rear_drive.setDirection(DcMotor.Direction.REVERSE);
        //right_front_drive.setDirection(DcMotor.Direction.REVERSE);;
        //set direction on motors

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intakearm.setDirection(DcMotorSimple.Direction.REVERSE);

        lift.setDirection(DcMotorSimple.Direction.REVERSE);

        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
