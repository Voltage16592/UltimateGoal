package TeamCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubSys_TestBot {
    DcMotor heavyArm;
    DigitalChannel cw_limitswitch;
    DigitalChannel cc_limitswitch;
    HardwareMap hardwareMap;

    SubSys_TestBot(){}

    public void init(HardwareMap hM){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.hardwareMap = hM;
        heavyArm = hardwareMap.get(DcMotor.class, "heavyArm");
        cw_limitswitch = hardwareMap.get(DigitalChannel.class, "cw_limitswitch");
        cc_limitswitch = hardwareMap.get(DigitalChannel.class, "cc_limitswitch");
    }

    public void moveHeavyArm(Gamepad gamepad1){
        if(!cw_limitswitch.getState() && gamepad1.right_stick_y > 0)
            heavyArm.setPower(0);
        else if(!cc_limitswitch.getState() && gamepad1.right_stick_y < 0)
            heavyArm.setPower(0);
        else
            heavyArm.setPower(ramp_Motor_Power(heavyArm.getPower(), gamepad1.right_stick_y));
    }

    private double ramp_Motor_Power(double current_Power, double desired_Power){
        double diff = desired_Power-current_Power;
        if(Math.abs(desired_Power) <0.2 || Math.abs(diff)<0.05)
            current_Power = desired_Power;
        else
            current_Power += (diff/(Math.abs(diff)))*0.05;
        return current_Power;
    }


}