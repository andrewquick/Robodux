package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

/**
 * Created by Quick's on 10/14/2017.
 */

@TeleOp(name = "Iowa")
@Disabled

public class Iowa extends OpMode {
    Hardware_robodux robot = new Hardware_robodux();

    boolean ready = true;
    boolean oneshot = false;

    int state = 1;

    int floor = -13;//Floor position for intake
    int low = 120;//Low intake position
    int storage = 440;//Storage Position for lift
    int highpos = 885;// High position for lift
    int top = 1160;//top position for lift

    double liftpower = .98;
    double delay = .1;

    double intakepos = 0;
    double clamppos = 0;

    double clampopen = .27;
    double clampmid = .2;
    double clampclosed = .065;

    double intakeopen = 0.2;
    double intakeallopen = .47;
    double intakeclosed = .065;
    double time;

    double error = 0;

    double reset = 0;
    boolean second_block = false;

    double rightpower = 0;
    double leftpower = 0;

    double range = 13;
    boolean mute = false;

    double pastleft = 0;
    double pastright = 0;
    double currentleft = 0;
    double currentright = 0;
    double changeleft = 0;
    double changeright = 0;

    boolean mode = false;
    boolean mode2 = false;


    public LynxI2cColorRangeSensor color1 = null;
    public LynxI2cColorRangeSensor color2 = null;
    @Override
    public void init() {

        robot.init(hardwareMap);
        color1 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color1");
        color2 = hardwareMap.get(LynxI2cColorRangeSensor.class, "color2");

        robot.liftr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.clampl.setPosition(.935);
        //robot.clampr.setPosition(.065);

        robot.liftr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    @Override
    public void loop() {

        if(color2.getDistance(DistanceUnit.MM) < 62 && (state == 1 || state ==5)){
            robot.jewel_arm.setPosition(0.53);
        }
        else{
            robot.jewel_arm.setPosition(.69);
        }

        if(state == 3 || state == 4 || state == 7 || state == 8){
            mute = true;
        }
        else{
            mute = false;
        }

        robot.right_jewel_stop.setPosition(1);
        robot.left_jewel_stop.setPosition(0);


        robot.intake_sevr.setPosition(1 - intakepos); //closed = 1
        robot.intake_sevl.setPosition(intakepos); //closed = 0

        robot.clampr.setPosition(clamppos);
        robot.clampl.setPosition(1 - clamppos);

        pastleft = leftpower;
        currentleft = -gamepad1.left_stick_y;
        changeleft = currentleft - pastleft;

        pastright = rightpower;
        currentright = -gamepad1.right_stick_y;
        changeright = currentright - pastright;

        if(gamepad1.left_bumper && mute == false){
            mode2 = false;
            if(mode == false){
                robot.right_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.left_front_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                mode = true;
            }
            rightpower = (-gamepad1.left_stick_y/3)-(gamepad1.left_stick_x/6);
            leftpower = (-gamepad1.left_stick_y/3)+(gamepad1.left_stick_x/6);
        }
        else if(mute == true){
            rightpower = 0;
            leftpower = 0;
        }
        else {
            mode = false;
            if(mode2 == false){
                robot.right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                mode2 = true;
            }
            rightpower = pastright + (changeright / 2.25);
            leftpower = pastleft + (changeleft / 2.25);
        }

        robot.right_front_drive.setPower(rightpower);
        robot.right_rear_drive.setPower(rightpower);
        robot.left_front_drive.setPower(leftpower);
        robot.left_rear_drive.setPower(leftpower);

        if(gamepad2.right_trigger > 0 && gamepad2.left_trigger == 0){
            robot.intakel.setPower(1);
            robot.intaker.setPower(1);
        }
        if(gamepad2.left_trigger > 0 ){
            robot.intakel.setPower(.25);
            robot.intaker.setPower(1);
        }
        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
            robot.intakel.setPower(-1);
            robot.intaker.setPower(-1);
        }
        if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
            robot.intakel.setPower(0);
            robot.intaker.setPower(0);
        }

        //STORE FIRST BLOCK
        if (gamepad2.left_bumper && ready && state ==1) {
            reset = 0;
            state = 3;
            oneshot = false;
            ready = false;
        }
        if (state == 2 && ready && reset == 0) {
            state = 3;
            oneshot = false;
            ready = false;
        }
        if (state == 3 && ready && !second_block) {
            state = 4;
            oneshot = false;
            ready = false;
        }
        if (state == 4 && ready) {
            state = 5;
            oneshot = false;
            ready = false;
        }
        //STORE SECOND BLOCK
        if (gamepad2.left_bumper && ready && (state == 5 || state ==12)) {
            state = 7;
            oneshot = false;
            ready = false;
        }
        if (state ==7 && ready) {
            state = 8;
            second_block = true;
            oneshot = false;
            ready = false;
        }
        if (state ==8 && ready) {
            state = 17;
            oneshot = false;
            ready = false;
        }
        // OPEN THE CLAMP IF SCORING AT BOTTOM
        if (state ==9 && ready && gamepad2.dpad_left) {
            state = 14;
            oneshot = false;
            ready = false;
        }
        // SCORE ONE BLOCK UP
        if ((state ==17 || state ==10 || state == 15 || state == 5 || state == 9) && ready && gamepad2.x) {
            state = 12;
            oneshot = false;
            ready = false;
        }
        // OPEN CLAMP TO SCORE
        if (state ==12 && ready && gamepad2.dpad_left) {
            state = 13;
            oneshot = false;
            ready = false;
        }
        // SCORE TWO BLOCKS UP
        if ((state ==17 || state == 12 || state == 15 || state == 5 || state == 9) && ready && gamepad2.y) {
            state = 10;
            oneshot = false;
            ready = false;
        }
        // OPEN CLAMPS AT TWO SCORING POSITION
        if (state ==10 && ready && gamepad2.dpad_left) {
            state = 11;
            oneshot = false;
            ready = false;
        }
        // SCORE THREE BLOCKS UP
        if ((state ==17 || state ==10 || state == 12 || state ==5 || state == 9) && ready && gamepad2.b && !gamepad2.start) {
            state = 15;
            oneshot = false;
            ready = false;
        }
        // OPEN CLAMPS AT THREE SCORING POSITION
        if (state ==15 && ready && gamepad2.dpad_left) {
            state = 16;
            oneshot = false;
            ready = false;
        }
        //BACK TO FLOOR
        if ((state ==10 || state ==12 || state == 15 || state ==5 || state ==17) && ready && gamepad2.a) {
            state = 9;
            oneshot = false;
            ready = false;
        }
        // RESET TO STATE ONE
        if ( ready && gamepad2.dpad_down) {
            reset = 1;
            state = 2;
            oneshot = false;
            second_block = false;
            ready = false;
        }
        if (state ==2 && ready && reset == 1) {
            state = 1;
            oneshot = false;
            ready = false;
        }
        if((state == 9 || state == 10 || state == 12 || state == 15) && gamepad2.start && ready){
            state = 5;
            oneshot = false;
            ready = false;
        }
        if (state == 1) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
            robot.liftr.setTargetPosition(floor);

            error = abs(floor - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

            intakepos = (intakeclosed);

            clamppos = (clampopen);

            if (robot.liftr.getCurrentPosition() > floor - range && robot.liftr.getCurrentPosition() < floor + range && (time + delay) < getRuntime()) {
                ready = true;
            }

        }

        if (state == 2) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
            robot.liftr.setTargetPosition(floor);
            error = abs(floor - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

            intakepos = (intakeclosed);

            clamppos = (clampclosed);

            if (robot.liftr.getCurrentPosition() > floor - range && robot.liftr.getCurrentPosition() < floor + range && (time + delay) < getRuntime()) {
                ready = true;
            }
        }

        if (state == 3) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(floor);

            error = abs(floor - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeopen);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > floor - range && robot.liftr.getCurrentPosition() < floor + range && (time + .2) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 4) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeopen);

                clamppos = (clampclosed);


                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 5) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeclosed);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 6) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeclosed);

                clamppos = (clampopen);

                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 7) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeopen);

                clamppos = (clampmid);

                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 8) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(floor);
            error = abs(floor - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeopen);

                clamppos = (clampmid);

                if (robot.liftr.getCurrentPosition() > floor - range && robot.liftr.getCurrentPosition() < floor + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 9) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(low);

            error = abs(low - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > low - range && robot.liftr.getCurrentPosition() < low + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 10) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(highpos);

            error = abs(highpos - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }
                intakepos = (intakeallopen);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > highpos - range && robot.liftr.getCurrentPosition() < highpos + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 11) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(highpos);
            error = abs(highpos - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampopen);

                if (robot.liftr.getCurrentPosition() > highpos - range && robot.liftr.getCurrentPosition() < highpos + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 12) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 13) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(storage);

            error = abs(storage - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampopen);

                if (robot.liftr.getCurrentPosition() > storage - range && robot.liftr.getCurrentPosition() < storage + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 14) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(low);
            error = abs(low - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampopen);

                if (robot.liftr.getCurrentPosition() > low - range && robot.liftr.getCurrentPosition() < low + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }

        if (state == 15) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(top);
            error = abs(top - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampclosed);

                if (robot.liftr.getCurrentPosition() > top - range && robot.liftr.getCurrentPosition() < top + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }


        if (state == 16) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
                robot.liftr.setTargetPosition(top);
            error = abs(top - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

                intakepos = (intakeallopen);

                clamppos = (clampopen);

                if (robot.liftr.getCurrentPosition() > top - range && robot.liftr.getCurrentPosition() < top + range && (time + delay) < getRuntime()) {
                    ready = true;
                }
        }
        if (state == 17) {

            if (oneshot == false) {
                time = getRuntime();
                ready = false;
                oneshot = true;
            }
            robot.liftr.setTargetPosition(low);
            error = abs(low - robot.liftr.getCurrentPosition());
            if(error > 8) {
                robot.liftr.setPower(liftpower);
            }
            else{
                robot.liftr.setPower(.5);
            }

            intakepos = (intakeopen);
            clamppos = (clampclosed);

            if (robot.liftr.getCurrentPosition() > low - range && robot.liftr.getCurrentPosition() < low + range && (time + delay) < getRuntime()) {
                ready = true;
            }
        }

    }

    @Override
    public void stop() {

    }
}
