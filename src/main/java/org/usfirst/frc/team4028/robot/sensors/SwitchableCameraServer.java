package org.usfirst.frc.team4028.robot.sensors;

// #region Imports
import java.nio.file.Files;
import java.nio.file.LinkOption;
import java.nio.file.Paths;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
// #endregion

// This class encapsulates interactions with the Diver / Operator Camera
public class SwitchableCameraServer {
	// =================================================================================================================
	// Define Enums for the Camera
	public enum CAMERA_CHOICE {
		UNKNOWN,
		DRIVER,
		CARRIAGE
	}
	
	// Note: Sonix (bottom)
	private static final String USB1_NAME = "driver camera";
	private static final String USB1_DEVICE_PATH =  "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.1:1.0-video-index0";
	
	// Note: LifeCAM (top)
	private static final String USB2_NAME = "carriage camera";	
	private static final String USB2_DEVICE_PATH = "/dev/v4l/by-path/platform-ci_hdrc.0-usb-0:1.2:1.0-video-index0";
	
    private static final int CAMERA_TCP_PORT = 1180;
      
    private UsbCamera _cameraRoboRioUsbPort1;
	private UsbCamera _cameraRoboRioUsbPort2;
	private CAMERA_CHOICE _currentCamera = CAMERA_CHOICE.UNKNOWN;
	
	private MjpegServer _rawVideoServer;
		
    // ==========================
    // Singleton Pattern
    // ==========================
    private static SwitchableCameraServer _instance = new SwitchableCameraServer();

	public static SwitchableCameraServer getInstance() {
		return _instance;
	}

	// private constructor
	private SwitchableCameraServer() {
		// set values to limit bandwidth usage
	 	final int width = 320; // 160; // 320; //640;
		final int height = 240; //90; //180; //480;
		final int frames_per_sec = 15; //10; //20; //15;
		
		_rawVideoServer = new MjpegServer("raw_video_server", CAMERA_TCP_PORT);    
		
		// =======================
		//  drivers camera (MegaPixel USB Camera)
		// ======================= 
		if (Files.exists(Paths.get(USB1_DEVICE_PATH), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("...camera1 exists");
			_cameraRoboRioUsbPort1 = new UsbCamera(USB1_NAME, USB1_DEVICE_PATH);
			_cameraRoboRioUsbPort1.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_cameraRoboRioUsbPort1.setExposureManual(5);
			_cameraRoboRioUsbPort1.setWhiteBalanceManual(50);
		}
		
		// =======================
		//  carriage camera (Microsoft LifeCam)
		// ======================= 
		if (Files.exists(Paths.get(USB2_DEVICE_PATH), LinkOption.NOFOLLOW_LINKS)) {
			System.out.println ("...camera2 exists");
			_cameraRoboRioUsbPort2 = new UsbCamera(USB2_NAME, USB2_DEVICE_PATH);
			_cameraRoboRioUsbPort2.setVideoMode(VideoMode.PixelFormat.kMJPEG, width, height, frames_per_sec);
			_cameraRoboRioUsbPort2.setExposureManual(60);
			_cameraRoboRioUsbPort2.setWhiteBalanceManual(50);
		}
		
		// set the initial camera
		if(_cameraRoboRioUsbPort1 != null) {
			setCamera(CAMERA_CHOICE.DRIVER);
		}
		else if(_cameraRoboRioUsbPort2 != null)	{
			setCamera(CAMERA_CHOICE.CARRIAGE);
		}
	}
	
	// swap between the 2 installed cameras
	public void toggle() {
		if (_currentCamera == CAMERA_CHOICE.DRIVER)	{
			setCamera(CAMERA_CHOICE.CARRIAGE);
		} else {
			setCamera(CAMERA_CHOICE.DRIVER);
		}
	}
	
	// switch to a particular camera
	/*
		The key concept here is to:
			1. avoid overloading the USB bus on the RoboRio by having both cameras "active" at the same time
			2. avoid consuming too much WiFi bandwidth by having both camera broadcasting images simultaneously

		To accomplish this we dynamically set the source of the MjpegServer to one of the cameras
	*/
	public void setCamera(CAMERA_CHOICE cameraChoice) {
		switch(cameraChoice) {
			case DRIVER:
			if ((_currentCamera == CAMERA_CHOICE.UNKNOWN) 
						|| (_currentCamera == CAMERA_CHOICE.CARRIAGE && _cameraRoboRioUsbPort1 != null)) {
					_rawVideoServer.setSource(_cameraRoboRioUsbPort1);
					_currentCamera = CAMERA_CHOICE.DRIVER;
					System.out.println ("current camera ==> chg'd to : " + _cameraRoboRioUsbPort1.getName());
				} else {
					System.out.println ("current camera ==> cannot be changed!");
				}
				break;
				
			case CARRIAGE:
				if (_currentCamera == CAMERA_CHOICE.DRIVER && _cameraRoboRioUsbPort2 != null) {
					_rawVideoServer.setSource(_cameraRoboRioUsbPort2);
					_currentCamera = CAMERA_CHOICE.CARRIAGE;		
					System.out.println ("current camera ==> chg'd to: " + _cameraRoboRioUsbPort2.getName());
				} else {
					System.out.println ("current camera ==> cannot be changed!");
				}
				break;

			case UNKNOWN:
				System.out.println ("current camera ==> Illegal value passed to setCamera!");
				break;
		}
	}
}
