package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class FlywheelTut extends OpMode {
    public DcMotorEx FlywheelTut;

    double highVelocity = 4000;

    double lowVelocity = 2000;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
