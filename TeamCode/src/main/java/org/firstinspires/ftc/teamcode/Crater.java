package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static com.qualcomm.hardware.bosch.BNO055IMU.AngleUnit.DEGREES;
import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/26/2017.
 */

@Autonomous(name = "Crater", group = "Blue")
public class Crater extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AQ4c5An/////AAAAGXHPS/VK/kM9p1fd0MoGsN5AKZHfSVWfqv0W1ccbbHtwCY0bnCb6rbBGQSzIqLtSUIzim4PQLMdvUS+6Q5g873CX538/L/4FY+HrtluUIIKTQefegsH27VRIlHgue83sg6mFSPUlbvXXND52Axl8eYEV6LHwru/dCqaAkHdT3dp18+l2nOS11sw5P8NeTiO06D5zg5NIB7L+qfGAYpnWrq8YBCJw2xcZFyZKcj+sqQzYJvGBbokn/dKmxwO5xteK3uvW908EcJ/1jGtY73MwnbkXO2QTRnVqN924N509GxecFAG15XY4UDZtNVS5LLB8Ik3u85o7K7nlQ47MUObGCicSIa0I2+tszy6VPsZjIikw";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private List cheese;

    OpenGLMatrix lastLocation = null;
    Hardware_robodux18 robot = new Hardware_robodux18();
    BNO055IMU imu;


    double initialheading = 0;
    double time = 0;
    boolean once = false;
    int i = 0;
    int position = 0;
    double timer = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        initVuforia();
        initTfod();

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.trapdoor.setPosition(0);
        robot.dumpright.setPosition(.945);
        robot.dumpleft.setPosition(.055);

        robot.latch.setPosition(.38);
        robot.trapdoor.setPosition(.365);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = DEGREES;

        imu.initialize(parameters);

        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }
        tfod.activate();
        sleep(75);
        robot.dumpright.setPosition(.935);
        robot.dumpleft.setPosition(.065);
        sleep(75);
        robot.lift.setPower(.5);
        sleep(300);
        robot.latch.setPosition(0);
        sleep(400);
        robot.lift.setPower(0);
        sleep(900);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //sleep(75);
        robot.lift.setPower(.8);
        //sleep(75);
        robot.lift.setTargetPosition(-2571);
        sleep(75);
        timer = getRuntime();

        while (getRuntime()<timer+1.35) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        position = 2;
                        //telemetry.addData("position", position);
                        //telemetry.update();
                    }
                }
            }
        }

        if(position == 2) {
            DriveStraight(1400, -.4);
            robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.lift.setPower(.35);
            sleep(2000);
            robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.lift.setTargetPosition(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            DriveStraight(447, .4);
            sleep(75);
            robot.intakearm.setPower(.7);
            sleep(75);
            robot.intakearm.setTargetPosition(-1900);
            sleep(2000);
        }
        if(position == 0){
            GyroTurn(-24,.5);
            sleep(75);
            timer = getRuntime();
            while (getRuntime()<timer+1.5) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            position = 1;
                            //telemetry.addData("position", position);
                            //telemetry.update();
                        }
                    }

                }
            }
            tfod.shutdown();
            sleep(75);
            if(position == 1){
                DriveStraight(1789, -.4);
                sleep(75);
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(.38);
                sleep(2000);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(2000);
            }
            if(position == 0){
                GyroTurn(24,.5);
                DriveStraight(1711, -.4);
                sleep(75);
                robot.lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.lift.setPower(.38);
                sleep(2000);
                robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.lift.setTargetPosition(0);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(2000);
            }
        }
    }
    public void GyroTurn(double target, double inputspeed) {
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double taccdist = 0;
        double tdeccdist = 0;
        double startangle = 0;
        boolean right = false;
        double startspeed = .12;
        double endspeed = .05;
        double speed = 0;
        double rightspeed = 0;
        double leftspeed = 0;
        double ediff = 0;
        double scalefactor = 525;
        double rightpower = 0;
        double leftpower = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;

        startangle = heading;

        if(target > startangle) {
            right = true;
            taccdist = startangle + 20;
            tdeccdist = target - 50;
        }
        if(target < startangle) {
            right = false;
            taccdist = startangle - 20;
            tdeccdist = target + 50;
        }
        while((heading < (target-1.5)) || (heading  > (target + 1.5))){
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;
            ediff = abs((robot.right_front_drive.getCurrentPosition() + robot.right_rear_drive.getCurrentPosition())/2)-abs((robot.left_front_drive.getCurrentPosition() + robot.left_rear_drive.getCurrentPosition())/2);

            if(right) {
                if (heading < taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading >= taccdist && heading <= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading > tdeccdist) {
                    speed = endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed)*.75;
                }
                rightspeed = -speed;
                leftspeed = speed;
            }
            if(!right){
                if (heading > taccdist) {
                    speed = startspeed + (heading / taccdist) * (inputspeed - startspeed);
                }
                if (heading <= taccdist && heading >= tdeccdist) {
                    speed = inputspeed;
                }
                if (heading < tdeccdist) {
                    speed = (endspeed + ((target - heading) / (target - tdeccdist)) * (inputspeed - endspeed))*.75;
                }
                rightspeed = speed;
                leftspeed = -speed;
            }
            rightpower = rightspeed * (1 - (ediff/scalefactor));
            leftpower = leftspeed * (1 + (ediff/scalefactor));

            robot.right_front_drive.setPower(rightpower);
            robot.right_rear_drive.setPower(rightpower);
            robot.left_front_drive.setPower(leftpower);
            robot.left_rear_drive.setPower(leftpower);
        }
        robot.right_front_drive.setPower(0);
        robot.right_rear_drive.setPower(0);
        robot.left_front_drive.setPower(0);
        robot.left_rear_drive.setPower(0);

    }
    public void DriveStraight(double distance, double inputpower) {

        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Orientation angle;
        double heading;

        double startingspeed = .09;
        double accdist = 200;
        double decdist = 300;
        double rightspeed = 0;
        double leftspeed = 0;
        double currentleft = 0;
        double currentright = 0;
        double switchpoint = 0;
        double originalheading = 0;

        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = -angle.firstAngle;
        originalheading = heading;

        if(inputpower < .3)
            decdist = 300;
        if(inputpower >= .3)
            decdist = 900*inputpower;

        while ((currentright < distance) || (currentleft < distance)) {
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = -angle.firstAngle;
            currentright = abs(robot.right_front_drive.getCurrentPosition());
            currentleft = abs(robot.left_front_drive.getCurrentPosition());

            if (distance >= (accdist + decdist)) {
                if (((currentright + currentleft) / 2) <= accdist) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / accdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + (currentleft / accdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / accdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / accdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                    }
                }
                if ((((currentright + currentleft) / 2) > accdist) && ((currentright + currentleft) / 2) < (distance - decdist)) {
                    if(inputpower >= 0) {
                        rightspeed = inputpower * (1 + ((heading - originalheading) / 40));
                        leftspeed = inputpower * (1 - ((heading - originalheading) / 40));
                    }
                    if(inputpower < 0){
                        rightspeed = inputpower * (1 - ((heading - originalheading) / 40));
                        leftspeed = inputpower * (1 + ((heading - originalheading) / 40));
                    }
                }
                if (((currentright + currentleft) / 2) >= (distance - decdist)) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / decdist) * (inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / decdist) * (inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / decdist) * (-inputpower - startingspeed)) * (1 - ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / decdist) * (-inputpower - startingspeed)) * (1 + ((heading - originalheading) / 40));
                    }
                }
            }
            if (distance < (accdist + decdist)) {
                switchpoint = (accdist + decdist) / 2;

                if (((currentright + currentleft) / 2) <= switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + (currentright / switchpoint) * (inputpower - startingspeed)* (1 + ((heading - originalheading) / 40)));
                        leftspeed = (startingspeed + (currentleft / switchpoint) * (inputpower - startingspeed)* (1 - ((heading - originalheading) / 40)));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + (currentright / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + (currentleft / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
                if (((currentright + currentleft) / 2) > switchpoint) {
                    if (inputpower > 0) {
                        rightspeed = (startingspeed + ((distance - currentright) / switchpoint) * (inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = (startingspeed + ((distance - currentleft) / switchpoint) * (inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                    if (inputpower < 0) {
                        rightspeed = -(startingspeed + ((distance - currentright) / switchpoint) * (-inputpower - startingspeed))* (1 + ((heading - originalheading) / 40));
                        leftspeed = -(startingspeed + ((distance - currentleft) / switchpoint) * (-inputpower - startingspeed))* (1 - ((heading - originalheading) / 40));
                    }
                }
            }
            robot.left_front_drive.setPower(leftspeed);
            robot.left_rear_drive.setPower(leftspeed);
            robot.right_front_drive.setPower(rightspeed);
            robot.right_rear_drive.setPower(rightspeed);
        }
        robot.left_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_rear_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }



    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        //int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                //"tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters();
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfodParameters.minimumConfidence = 0.9;
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}

