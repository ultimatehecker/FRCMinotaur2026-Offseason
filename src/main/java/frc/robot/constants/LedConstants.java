package frc.robot.constants;

import frc.minolib.hardware.MinoCANDevice;
import frc.robot.Constants;

public class LedConstants {
    public static final MinoCANDevice kCandle = new MinoCANDevice(28, Constants.kCANivoreBus);

    public static final int kCANdleDebugLength = 7;
    public static final int kTotalLedLength = 111 + kCANdleDebugLength;
}
