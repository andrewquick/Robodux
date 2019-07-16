package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.abs;

/**
 * Created by Quick's on 8/28/2017.
 */

@Autonomous(name = "Straight")
@Disabled

public class Drive_Straight extends LinearOpMode {


    Hardware_robodux robot = new Hardware_robodux();


    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        waitForStart();
        DriveStraight(700, .5);
    }

        public void DriveStraight(double distance, double inputpower) {

            double rightpower = 0;
            double leftpower = 0;
            double leftposition = -920;
            double rightposition = -910;
            double accmult;
            double deccmult;
            double diff;
            double scalingfactor = 600;
            double accdist = 132.2;
            double oneshot = 0;

        while (((((abs(leftposition + rightposition)) / 2) + 3) < distance) || ((((abs(leftposition + rightposition)) / 2) - 3) > distance)) {


            if (distance < 0 && oneshot == 0) {
                inputpower = inputpower * (-1);
                oneshot = 1;
            }

            diff = rightposition - leftposition;

            if (diff > 0 && inputpower > 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff < 0 && inputpower > 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);

            }

            if (diff > 0 && inputpower < 0) {
                rightpower = inputpower;
                leftpower = inputpower + (diff / scalingfactor);
            }

            if (diff < 0 && inputpower < 0) {
                rightpower = inputpower - (diff / scalingfactor);
                leftpower = inputpower;
            }

            if (diff == 0.0) {
                rightpower = inputpower;
                leftpower = inputpower;
            }

            if (abs(distance) < (2 * accdist)) {
                accmult = 1;
                deccmult = 1;
            } else {
                accmult = (0.1 / (abs(inputpower))) + ((abs((rightposition + leftposition) / 2)) / accdist);
                if (accmult > 1)
                    accmult = 1;

                if ((abs(distance - ((rightposition + leftposition) / 2))) < accdist) {
                    deccmult = ((0.1 / abs(inputpower)) + ((abs(distance - ((rightposition + leftposition) / 2))) / accdist));
                    if (deccmult > 1)
                        deccmult = 1;
                } else
                    deccmult = 1;
            }
            rightpower = rightpower * accmult * deccmult;
            leftpower = leftpower * accmult * deccmult;

            //set motors to motor powers
            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);


            telemetry.addLine()
                    .addData("rightpower", rightpower);
            telemetry.addLine()
                    .addData("leftpower", leftpower);
        }
        }
        //set powers to zero
        //reset encoders
}
