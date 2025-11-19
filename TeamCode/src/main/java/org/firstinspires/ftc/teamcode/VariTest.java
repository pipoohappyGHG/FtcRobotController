package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class VariTest extends OpMode {
    @Override
    public void init() {
        int Test = 67;
        double Komma = 6.7;
        boolean GarantedFalse = true;
        String Name = "Xaver";

        telemetry.addData("Test", Test);
        telemetry.addData("Kommazahl", Komma);
        telemetry.addData("always false", GarantedFalse);
        telemetry.addData("Name", Name);
    }

    @Override
    public void loop() {

    }
}

