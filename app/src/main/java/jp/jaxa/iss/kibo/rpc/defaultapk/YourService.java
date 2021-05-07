package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import net.sourceforge.zbar.Config;
import net.sourceforge.zbar.Image;
import net.sourceforge.zbar.ImageScanner;
import net.sourceforge.zbar.Symbol;
import net.sourceforge.zbar.SymbolSet;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Kinematics;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import static org.opencv.android.Utils.matToBitmap;

public class YourService extends KiboRpcService
{
	int Pattern = 0;				// Store pattern value.
	boolean QRCodeFinish = false,
			ARCodeFinish = false,
			CircleFinish = false;	// State of QR, AR event and circle detection event.

	final Point Point_A = new Point(11.21, -9.8, 4.79);	//	Point of A
	final Quaternion Quaternion_A = new Quaternion(0, 0, -0.707f, 0.707f);	// Quaternion of A
	Point Point_A_Prime = new Point(0, 0, 0);				//	Point of A prime
	Point Point_Target = new Point(0, -10.585, 0);		//	Point of Target
	Point Point_Shift1 = new Point(-0.16966, 0, -0.02683);	//	Laser distance shift (1: AR Intersection Method).
	Point Point_Shift2 = new Point(-0.17866, 0, -0.13483);	//	Laser distance shift (2: HC Average Method).

	@Override
    protected void runPlan1()
	{
		api.startMission();
        Log.d("Robot[State]: ", "Starting the mission");

		Log.d("Robot[State]: ", "Laser is on");
		api.laserControl(true);


		PointA_Event();


		Log.d("Robot[State]: ", "Aim to target point");
        moveTo(Point_A.getX(), Point_A.getY(), Point_A.getZ(),	Point_Target.getX() + Point_Shift2.getX(),
																Point_Target.getY() + Point_Shift2.getY(),
																Point_Target.getZ() + Point_Shift2.getZ());

        Log.d("Robot[State]: ", "Take a snapshot");
        api.takeSnapshot();

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		moveTo(Point_A, new Quaternion(0.707f, -0.707f, 0, 0)); // Rotate Test!!!
//
//		long timeStart = SystemClock.elapsedRealtime();
//		while(SystemClock.elapsedRealtime() - timeStart < 5000);
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		Log.d("Robot[State]: ", "Move to A-B point");
        moveTo(10.46f, -8.65f, 4.65f, 0f, 0f, 0.707f, 0.707f);
		Log.d("Robot[State]: ", "Laser is off");
		api.laserControl(false);
		Log.d("Robot[State]: ", "Move to B point");
		moveTo(10.6f, -8.0f, 4.5f, 0f, 0f, -0.707f, 0.707f);

		finishMission();
    }
    @Override
    protected void runPlan2()
    {
    }
    @Override
    protected void runPlan3()
    {
    }
	public void moveTo(Point point, Quaternion quaternion)
	/* Re-check api.moveTo function is successful or until max number of counter. */ {
		Result result;
		int count = 0, max_count = 2;
		do
		{
			result = api.moveTo(point, quaternion, true);
			count++;
		}
		while (!result.hasSucceeded() && count < max_count);
	}
	public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
	/* Convert float number in argument to object of point and quaternion. */ {
		Point point = new Point(px, py, pz);
		Quaternion quaternion = new Quaternion(qx, qy, qz, qw);
		moveTo(point, quaternion);
	}
	public void finishMission()
	/* Re-check api.reportMissionCompletion, The cause is sometimes unresponsive.*/ {
		Log.d("reportMissionCompletion[State]: ", "Starting");
		boolean ReportFinish = false;
		while(!ReportFinish)
		{
			try
			{
				api.reportMissionCompletion();
				ReportFinish = true;
			}
			catch (Exception error)
			{
				Log.e("reportMissionCompletion[State]: ", "Failure");
				Log.e("reportMissionCompletion[Error]: ", error.getMessage());
			}
		}
		Log.d("reportMissionCompletion[State]: ", "Finished");
	}
    public double[] StringParseToDouble(String contents)
    /* Split the data obtained from the QR code into double numbers. */ {
        Log.d("StringParseToDouble[State]: ", "Starting");
        Log.d("StringParseToDouble[Input]: ", "" + contents);
        double pattern = 0, point_x = 0, point_y = 0, point_z = 0;
        try
        {
            // Assume : {"p":5,"x":11.07,"y":-9.80,"z":5.49}
            String contents_cut = contents.substring(1, 35);      // Result > p":5,"x":11.07,"y":-9.80,"z":5.49
            String[] commaParse = contents_cut.split(",");  // Result > "p":5 , "x":11.07 , "y":-9.80 , "z":5.49
            String[] pParse = commaParse[0].split(":");     // Result > "p" , 5
            String[] xParse = commaParse[1].split(":");     // Result > "x" , 11.07
            String[] yParse = commaParse[2].split(":");     // Result > "y" , -9.80
            String[] zParse = commaParse[3].split(":");     // Result > "z" , 5.49

            // String to double value convert.
            pattern = Double.parseDouble(pParse[1]);
            point_x = Double.parseDouble(xParse[1]);
            point_y = Double.parseDouble(yParse[1]);
            point_z = Double.parseDouble(zParse[1]);
            Log.d("StringParseToDouble[Info]: ", "p: " + pattern + ", x: " + point_x + ", y: " + point_y + ", z: " + point_z);
        }
        catch (Exception error)
        {
            Log.e("StringParseToDouble[State]: ", "Failure");
            Log.e("StringParseToDouble[Error]: ", error.getMessage());
        }
        Log.d("StringParseToDouble[State]: ", "Finished");
        // Double return : pattern, point x, point y, point z
        return new double[]{pattern, point_x, point_y, point_z};
    }
    public Mat distortionFix(Mat sourceIn)
    /* Used to correct the distortion problem of matrix image. */ {
        Log.d("distortionFix[State]: ", "Starting");
        // Set dimension of matrix data.
        Mat sourceOut = new Mat(1280, 960, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat dCoefficient = new Mat(1, 5, CvType.CV_32FC1);
        int row = 0, col = 0;

        // Set matrix parameter from Array.
        final double[] cameraMatrixList =
        {
            567.229305D,    0.00000D, 659.077221D,
                0.0000D, 574.192915D, 517.007571D,
                0.0000D,    0.00000D,       1.00D
        };
		final double[] dCoefficientList = {-0.216247D, 0.03875D, -0.010157D, 0.001969D, 0.0D};
        cameraMatrix.put(row, col, cameraMatrixList);
        dCoefficient.put(row, col, dCoefficientList);

		try
        {
            // Take OpenCV function.
            Imgproc.undistort(sourceIn, sourceOut, cameraMatrix, dCoefficient);
        }
        catch(Exception error)
        {
            // Prevent errors by return the source image.
            Log.e("StringParseToDouble[State]: ", "Failure");
            Log.e("StringParseToDouble[Error]: ", error.getMessage());
            sourceOut = sourceIn;
        }
        Log.d("distortionFix[State]: ", "Finished");
        return sourceOut;
    }
	public Rect CustomCrop(int originWidth, int originHeight, int offsetLeft, int offsetRight)
	/* Calculate crop image, requiring only the offset of x (width), And the height is under automatic 4:3 ratio, and the y bottom without offset. */ {
		Log.d("CustomCrop[State]: ", "Starting");
		int offsetTop = (offsetLeft+offsetRight)/4*3; 		// Calculate for offset of y top
		int width	= originWidth-offsetLeft-offsetRight;   // Calculate for width of image
		int height 	= originHeight-offsetTop;          		// Calculate for height of image
		Log.d("CustomCrop[State]: ", "Finished");
		return new Rect(offsetLeft, offsetTop, width, height);
	}
    public double[] Intersection(double[][] p)
    /*  Calculates the intersection of an irregular rectangle, the data is coordinate and average of line length. */ {
        /* Input Format :
		    [X 1 Line 1][Y 1 Line 1]
		    [X 2 Line 1][Y 2 Line 1]
		    [X 1 Line 2][Y 1 Line 2]
		    [X 2 Line 2][Y 2 Line 2]
		*/
    	double[] dataOut = new double[3];

        // Analyze from linear line equation.
        double a = (p[1][0] - p[0][0]) * (p[3][0] - p[2][0]);
        double b = (p[1][0] - p[0][0]) * (p[3][1] - p[2][1]);
        double c = (p[3][0] - p[2][0]) * (p[1][1] - p[0][1]);
        dataOut[0] = (a * p[0][1] + b * p[2][0] - a * p[2][1] - c * p[0][0]) / (b - c);
        dataOut[1] = ((p[1][1] - p[0][1]) * (dataOut[0] - p[0][0]) / (p[1][0] - p[0][0])) + p[0][1];

        // Analyze from Pythagorean Theorem.
        double lineX1 = Math.pow(p[0][0] - p[1][0], 2);
        double lineY1 = Math.pow(p[0][1] - p[1][1], 2);
        double lineX2 = Math.pow(p[3][0] - p[2][0], 2);
        double lineY2 = Math.pow(p[3][1] - p[2][1], 2);
        double lineAvg = (Math.sqrt(lineX1 + lineY1) + Math.sqrt(lineX2 + lineY2)) / 2.0D;
        dataOut[2] = lineAvg;

        // return data format 0: center of x, 1: center of y, 2: average of line length.
        return dataOut;
    }
    public void PointA_Event()
    /* All events that take position at the point of A include QR Code Reading, AR Code Reading and Target position analyzing. */ {
		/* Set parameter related to image from navigation camera. */
		final int widthCut = 840;			// Width you need cut off.
        final int originalWidth = 1280;    // Original width of Nav Camera.
        final int originalHeight = 960;    // Original height of Nav Camera.
        final int finalWidth = originalWidth - widthCut; // Result width after the process.
        final int finalHeight = finalWidth * 3/4;        // Result height after the process.

		/* Set parameter about AR Code. */
		Mat IDs = new Mat(); 						// Variable store ID each of AR code. : Matrix format
		List<Mat> Corners = new ArrayList<>(); 	// Variable store four corner each of AR Code.
		Dictionary Dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250); // Standard AR code format according rule book.
		int[] AR_ID = new int[]{0, 0, 0, 0};		// Variable store ID each of AR code. : Array of Int format
		double[][] AR_Center = new double[4][2]; 	// Variable store center each of AR tag.
		/* Set parameter about QR Code. */
		Bitmap bitmapSrc = Bitmap.createBitmap(finalWidth, finalHeight, Bitmap.Config.ARGB_8888); // Bitmap image according to the requirement of QR reading.
		String QR_Info = null; 						// Content from QR Code reader.
		/* Set parameter about target tag */
		double PixelToMeter = 0;					// Ratio of AR dimension.
		/* Set general parameter */
		int loopCount = 0; 				// Variable of loop counter.
		final int loopCountMax = 5;		// Variable of max loop count.

		while(!CircleFinish && loopCount < loopCountMax)
        { /* The condition is defined with success ar code reading and the number of loops (the AR code must be read before the AR code can be read). */
            Log.d("Move&GetImage[State]:", " Starting");
            long timeStart = SystemClock.elapsedRealtime();

            // First step : Movement to point A positionn -> Get image form Nav Camera -> Get realtime position of robot.
            moveTo(Point_A, Quaternion_A);
            Mat matSrc = new Mat(api.getMatNavCam(), CustomCrop(originalWidth, originalHeight, widthCut/2, widthCut/2));
            Kinematics Robot = api.getTrustedRobotKinematics();
			Log.d("Move&GetImage[State]:", " Finished");
			Log.d("Move&GetImage[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);

			// Second step (QR code event) : Get image form Nav Camera -> QR Code decoder -> Send data of QR code to judge.
			if(!QRCodeFinish)
			{
				try
				{
					Log.d("QR[State]:", " Starting");
					timeStart = SystemClock.elapsedRealtime();

					matToBitmap(matSrc, bitmapSrc, false); 					// Convert mat image to bitmap image.
					int[] pixel = new int[bitmapSrc.getWidth() * bitmapSrc.getHeight()];  // Calculate image dimension.
					bitmapSrc.getPixels(pixel, 0, bitmapSrc.getWidth(), 0, 0, bitmapSrc.getWidth(), bitmapSrc.getHeight());
					Image QR_code = new Image(bitmapSrc.getWidth(), bitmapSrc.getHeight(), "RGB4");
					QR_code.setData(pixel);

					ImageScanner reader = new ImageScanner();	// Set parameter about reader.
					reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
					reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
					reader.scanImage(QR_code.convert("Y800"));

					SymbolSet Symbols = reader.getResults(); // Get result form reader.
					for (Symbol Sym : Symbols)
					{
						QR_Info = Sym.getData();
						Log.e("QR[State]:", " Detected");
						Log.d("QR[Data][Raw]:", " " + QR_Info);
					}
				}
				catch (Exception error)
				{
					Log.e("QR[State]:", " Not detected");
					Log.e("QR[Error]: ", error.getMessage());
				}
				finally
				{
					if(QR_Info != null)
					{ /* The condition is QR Code reading successful. */
						api.sendDiscoveredQR(QR_Info); // Send data of QR code to judge.

						double[] Buf = StringParseToDouble(QR_Info);
						Pattern = (int) Buf[0];
						Point_A_Prime = new Point(Buf[1], Buf[2], Buf[3]);
						Log.d("QR[Data][Pattern]:", " " + Pattern);
						Log.d("QR[Data][A_Prime]:", " X: " + Point_A_Prime.getX() + ", Y: " + Point_A_Prime.getY() + ", Z: " + Point_A_Prime.getZ());
						QRCodeFinish = true; // Set new condition.
					}
					Log.d("QR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
				}
			}

			// Third step (AR code event) : Get image form Nav Camera -> AR Code decoder -> Calculate target position.
			boolean AR_ON = false;	// ON-OFF AR event function
			if(!ARCodeFinish && QRCodeFinish && AR_ON)
			{
				timeStart = SystemClock.elapsedRealtime();
				try
				{
					Log.d("AR[State]:", " Starting");
					timeStart = SystemClock.elapsedRealtime();

					Aruco.detectMarkers(matSrc, Dict, Corners, IDs); 	// AR tag detector
					AR_ID = new int[] 									// Put ID of AR Code to Array of int.
					{
						(int) IDs.get(0, 0)[0],
						(int) IDs.get(1, 0)[0],
						(int) IDs.get(2, 0)[0],
						(int) IDs.get(3, 0)[0]
					};
					Log.d("AR[State]:", " Detected");
				}
				catch (Exception error)
				{
					Log.e("AR[State]:", " Not detected");
					Log.e("AR[Error]: ", error.getMessage());
				}
				finally
				{
					if (AR_ID[0] != 0 && AR_ID[1] != 0 && AR_ID[2] != 0 && AR_ID[3] != 0)
					{ /* The condition is four AR Code reading successful. */
						Log.d("TargetCal[State]:", " Starting");
						long timeStart2 = SystemClock.elapsedRealtime();

						for (int i = 0; i < 4; i++)
						{
							double[][] Corner =		// Put each AR tag corners to 2d-array of double.
							{
								{(int) Corners.get(i).get(0, 0)[0], (int) Corners.get(i).get(0, 0)[1]},
								{(int) Corners.get(i).get(0, 2)[0], (int) Corners.get(i).get(0, 2)[1]},
								{(int) Corners.get(i).get(0, 1)[0], (int) Corners.get(i).get(0, 1)[1]},
								{(int) Corners.get(i).get(0, 3)[0], (int) Corners.get(i).get(0, 3)[1]}
							};
							double[] AR_Point = Intersection(Corner); 	// Calculate center of AR tag.
							int Index = -1;								// Set initial index
						  		 if (AR_ID[i] == 1) Index = 0;			// Origin coordinate of Line 1
							else if (AR_ID[i] == 2) Index = 2;			// Origin coordinate of Line 2
							else if (AR_ID[i] == 3) Index = 1;			// Final coordinate of Line 1
							else if (AR_ID[i] == 4) Index = 3;			// Final coordinate of Line 2
							AR_Center[Index][0] = AR_Point[0];			// return x point of AR tag
							AR_Center[Index][1] = AR_Point[1];			// return y point of AR tag
						}
						double[] Buf = Intersection(AR_Center); 			// Calculate target point from AR tag.
						Buf[0] = (originalWidth - finalWidth)/2D + Buf[0];	// Reference the original width image size.
						Buf[1] = originalHeight - finalHeight    + Buf[1];	// Reference the original height image size.
						PixelToMeter = Buf[2] / 0.2398207664D;				// Constant value is actual AR tag diagonal size
						Log.d("TargetCal[Data][Raw_Dif}:", " X: " + (640-Buf[0]) + " Y: " + (480-Buf[1]));
						Log.d("TargetCal[Data][Ratio}:", " " + PixelToMeter);
//						PixelToMeter = 618.3450759928269D;					// Constant ratio by won-spaceY.
						Point Point_Robot = Point_A;						// Temporary coordinate of target.
						if(Robot != null)									// Re-check get kinematic is successful.
						{
							Point_Robot = Robot.getPosition();
							Log.d("getKinematics[Data]:"," X: " + Point_Robot.getX() + ", Y: " + Point_Robot.getY() + ", Z: " + Point_Robot.getZ());
							Log.d("getKinematics[State]:"," Finished");
						}
						/*	->	*/	Point_Robot = Point_A; 	// Fix position Test!!!
						double X_dif = (originalWidth  / 2.0D - Buf[0]) / PixelToMeter;	// The x-axis distance between the robot and the target : meter unit
						double Y_dif = (originalHeight / 2.0D - Buf[1]) / PixelToMeter;	// The y-axis distance between the robot and the target : meter unit
						Point_Target = new Point(Point_Robot.getX() - X_dif, Point_Target.getY(), Point_Robot.getZ() - Y_dif);	// Calculate target position on ISS
						/* Data Logger */
						Log.d("TargetCal[Data][Dif]:", " X: " + X_dif + ", Y: " + Y_dif);
						Log.d("TargetCal[Data][Robot_Point]:", " X: " + Point_Robot.getX() + ", Y: " + Point_Robot.getY() + ", Z: " + Point_Robot.getZ());
						Log.d("TargetCal[Data][Point]:", " X: " + Point_Target.getX() + ", Y: " + Point_Target.getY() + ", Z: " + Point_Target.getZ());
						Log.d("TargetCal[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart2));
						ARCodeFinish = true; // Set new condition.
					}
					Log.d("AR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
				}
			}

			// Fourth step (Hough circle detection) : Get image form Nav Camera -> Circle detector -> Calculate target position.
			if(!CircleFinish && QRCodeFinish)
			{
				Log.e("HC[State]: ", "Starting");
				timeStart = SystemClock.elapsedRealtime();
				/* Variables are used to store matrix images of each process. */
				Mat gray = new Mat();
				Mat blur = new Mat();
				Mat circles = new Mat();

				try
				{
					Imgproc.cvtColor(matSrc, matSrc, Imgproc.COLOR_GRAY2BGR); // 3UC1 Image convert to BGR image.
					Imgproc.cvtColor(matSrc, gray, Imgproc.COLOR_BGR2GRAY);   // BGR Image convert to grayscale image.
					Imgproc.medianBlur(gray, blur, 5);					// Median blur process
					Imgproc.HoughCircles(blur, circles, Imgproc.HOUGH_GRADIENT, 1, 5, 1000, 20, 30, 50);
					//	1.Input image 2.Output data 3.Type of Method 4.Ratio of raw image
					//	5.Minimum distance of center to center, 6.Canny edge value, 7.Threshold value 8.Minimum circle radius 9.Maximum circle radius
					Log.d("HC[Number]: ", "" + circles.cols()); 	// "circles.cols()" is number of circles to detect.
					double x_avg = 0, y_avg = 0, radius = 0;				// x, y position and radius (In average value).
					for (int i = 0; i < circles.cols(); i++ )
					{
						double[] data = circles.get(0, i);	// Get x, y, r data
						x_avg  += Math.round(data[0]);				// Get X position at i index
						y_avg  += Math.round(data[1]);				// Get Y position at i index
						radius += Math.round(data[2]);				// Get Radius at i index
						Log.d("HC[Point][" + i + "]:"," X: " + Math.round(data[0]) + ", Y: " +  Math.round(data[1]) + ", R: " + Math.round(data[2]));
					}
					x_avg = (originalWidth - finalWidth)/2D + (x_avg / circles.cols());	// Reference the original width image size.
					y_avg = originalHeight - finalHeight    + (y_avg / circles.cols());	// Reference the original height image size.
					PixelToMeter = radius / circles.cols() / 0.075;						// Using constant value of target tag radius.
					double X_dif = (originalWidth  / 2.0D - x_avg) / PixelToMeter;			// The x-axis distance between the robot and the target : meter unit
					double Y_dif = (originalHeight / 2.0D - y_avg) / PixelToMeter;			// The y-axis distance between the robot and the target : meter unit
					Point_Target = new Point(Point_A.getX() - X_dif, Point_Target.getY(), Point_A.getZ() - Y_dif);	// Calculate target position on ISS
					/* Data Logger */
					Log.d("HC[Data][Ratio]:", " " + PixelToMeter);
					Log.d("HC[Data][Dif]:", " X: " + X_dif + ", Y: " + Y_dif);
					Log.d("HC[Data][Point]:", " X: " + Point_Target.getX() + ", Y: " + Point_Target.getY() + ", Z: " + Point_Target.getZ());
					CircleFinish = true; // Set new while() condition
				}
				catch (Exception error)
				{
					Log.e("HC[State]: ", "Failure");
					Log.e("HC[Error]: ", error.getMessage());
				}
				Log.d("HC[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
			}
			Log.d("A_Event[State]:", " Finished");
			Log.d("A_Event[Count]:", " " + loopCount);
			loopCount++;
        }
    }
    public void moveTo(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des)
    /* Primeval function waiting to update. */ {
        double dx = x_des-x_org;
        double dy = y_des-y_org;
        double dz = z_des-z_org;
        double magnitude = Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
        double x_unit = dx/magnitude;
        double y_unit = dy/magnitude;
        double z_unit = dz/magnitude;

        double[][] matrix =
        {
            {1, 0, 0},
            {x_unit, y_unit, z_unit}
        };

        double x = matrix[0][1]*matrix[1][2] - matrix[1][1]*matrix[0][2];
        double y = matrix[0][2]*matrix[1][0] - matrix[1][2]*matrix[0][0];
        double z = matrix[0][0]*matrix[1][1] - matrix[1][0]*matrix[0][1];
        double i = matrix[1][0]-matrix[0][0];
        double j = matrix[1][1]-matrix[0][1];
        double k = matrix[1][2]-matrix[0][2];
        double q = Math.sqrt(x*x + y*y + z*z);
        double p = Math.sqrt(i*i + j*j + k*k);
        double theta = Math.acos((2 - p*p) / 2);

        double a = Math.sin(theta/2)*x/q;
        double b = Math.sin(theta/2)*y/q;
        double c = Math.sin(theta/2)*z/q;
        double w = Math.cos(theta/2);

        Log.d("[Qua]_X:", " "+a);
        Log.d("[Qua]_Y:", " "+b);
        Log.d("[Qua]_Z:", " "+c);
        Log.d("[Qua]_W:", " "+w);

        moveTo((float)x_org, (float)y_org, (float)z_org, (float)a, (float)b, (float)c, (float)w);
    }
}
//                                                                                                                                       lll         lll
//                                                                                                                                        lll       lll
//		lll      lllll      lll lllllllllllllll llllll      lll         llllllllllll llllllllllll    lllll       llllllllllll llllllllllll lll     lll
//		lll     lll lll     lll lll         lll lll lll     lll         lll          lll      lll   lll lll      lll          lll           lll   lll
//       lll   lll   lll   lll  lll         lll lll  lll    lll         lll          lll      lll  lll   lll     lll          lll            lll lll
//       lll   lll   lll   lll  lll         lll lll   lll   lll lllllll llllllllllll llllllllllll lllllllllll    lll          llllllllllll    lllll
//        lll lll     lll lll   lll         lll lll    lll  lll                  lll lll         lll       lll   lll          lll              lll
//        lll lll     lll lll   lll         lll lll     lll lll                  lll lll        lll         lll  lll          lll              lll
//         lllll       lllll    lllllllllllllll lll      llllll         llllllllllll lll       lll           lll llllllllllll llllllllllll     lll
//
//         lllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll

