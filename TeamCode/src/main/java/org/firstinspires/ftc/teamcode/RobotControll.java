package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Set;

@TeleOp
public class RobotControll extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        // runs 50x* a second
        double speedForward = gamepad1.right_stick_y / 2.0;
        double Difference = gamepad1.right_stick_x - gamepad1.right_stick_y;
        double sumTriggers = gamepad1.right_trigger + gamepad1.left_trigger;

        telemetry.addData("rs x", gamepad1.right_stick_x);
        telemetry.addData("rs y", speedForward);
        telemetry.addData("rs a button", gamepad1.a);
        telemetry.addData("ls x", gamepad1.left_stick_x);
        telemetry.addData("ls y", gamepad1.left_stick_y);

        telemetry.addData("B", gamepad1.b);
        telemetry.addData("A", gamepad1.a);

        telemetry.addData("dif x",Difference);

        telemetry.addData("sumTriggers", sumTriggers);



    }

    /*

     */

}
