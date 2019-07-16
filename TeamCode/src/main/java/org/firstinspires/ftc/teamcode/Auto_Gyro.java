/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name= "Auto_Gyro")
@Disabled
public class Auto_Gyro extends LinearOpMode {

    Hardware_robodux robot = new Hardware_robodux();

    @Override
    public void runOpMode() throws InterruptedException {

        while (opModeIsActive()){

                robot.init(hardwareMap);

                robot.Testmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.Testmotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                idle();

                robot.Testmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.Testmotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                waitForStart();






        }


    }
}
*/