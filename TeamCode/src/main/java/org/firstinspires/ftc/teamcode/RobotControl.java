package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Disabled
@TeleOp
public class RobotControl extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Goodnight", "Derek"); // Love you Derek
    }

    @Override
    public void loop() {

        // This is an example of a single line comment.
        // The computer completely ignores these.

        /*
         Anything between this astrict
        and the slash
        will be completely ignored.
        */

    }
}
