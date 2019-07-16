/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Quick's on 8/27/2017.
 */
/*
@TeleOp (name = "Encoder")

public class Encoder extends OpMode {

    Hardware_robodux robot = new Hardware_robodux();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.Testmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Testmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.Testmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Testmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addLine()
                .addData("Encoder", robot.Testmotor.getCurrentPosition());

        robot.Testmotor.setPower(.2);
    }
}
*/