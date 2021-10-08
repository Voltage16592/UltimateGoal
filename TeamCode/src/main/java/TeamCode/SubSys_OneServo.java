package TeamCode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SubSys_OneServo {
    CRServo beltServo;
    DcMotor beltMotor;
    DcMotor leftWhoosh;
    DcMotor rightWhoosh;
    HardwareMap hardwareMap;

    SubSys_OneServo(){}

    public void init(HardwareMap hM){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        this.hardwareMap = hM;
        beltServo = hardwareMap.get(CRServo.class, "beltServo");
        beltMotor = hardwareMap.get(DcMotor.class, "beltMotor");
        leftWhoosh = hardwareMap.get(DcMotor.class, "leftWhoosh");
        rightWhoosh = hardwareMap.get(DcMotor.class, "rightWhoosh");


    }


    public void moveServo(Gamepad gamepad1){
        if(gamepad1.left_bumper){ //to close mouth
            beltServo.setPower(-1);
        } else if(gamepad1.right_bumper){ //to open mouth
            beltServo.setPower(1);
        } else {beltServo.setPower(0);}
    }

    public void moveMotor(Gamepad gamepad1){ beltMotor.setPower(gamepad1.right_stick_y/2); }

    public void pewpew(Gamepad gamepad1){ leftWhoosh.setPower(-gamepad1.left_stick_y); rightWhoosh.setPower(gamepad1.left_stick_y);}



}
