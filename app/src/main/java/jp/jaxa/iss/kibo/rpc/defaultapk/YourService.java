package jp.jaxa.iss.kibo.rpc.defaultapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.*;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;
import net.sourceforge.zbar.*;
import org.opencv.aruco.*;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.*;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import static org.opencv.android.Utils.matToBitmap;
import java.util.*;

//      [Team member: 1.Tee:Tula 2.Earth:Teerachot 3.Rew:Siraphop]                                                                       lll         lll
//                                                                								                                             lll       lll
//      lll      lllll      lll lllllllllllllll llllll      lll         llllllllllll llllllllllll    lllll       llllllllllll llllllllllll lll     lll
//      lll     lll lll     lll lll         lll lll lll     lll         lll          lll      lll   lll lll      lll          lll           lll   lll
//       lll   lll   lll   lll  lll         lll lll  lll    lll         lll          lll      lll  lll   lll     lll          lll            lll lll
//       lll   lll   lll   lll  lll         lll lll   lll   lll lllllll llllllllllll llllllllllll lllllllllll    lll          llllllllllll    lllll
//        lll lll     lll lll   lll         lll lll    lll  lll                  lll lll         lll       lll   lll          lll              lll
//        lll lll     lll lll   lll         lll lll     lll lll                  lll lll        lll         lll  lll          lll              lll
//         lllll       lllll    lllllllllllllll lll      llllll         llllllllllll lll       lll           lll llllllllllll llllllllllll     lll

public class YourService extends KiboRpcService
{
	int Pattern = 0;				//	Store pattern value.
	boolean QRCodeFinish	= false,
			ARCodeFinish 	= false,	//	State of QR and AR event.
			FirstRun	 	= true;
	final Quaternion Zero_Quaternion	= new Quaternion(0, 0, 0, 0);				//	No rotate quaternion
	Quaternion Quaternion_Target 		= new Quaternion(0, 0, 0, 0);				//	Quaternion of target point.
	//**	final Point Point_A 	= new Point(11.21-0.48, -9.8, 4.79);	//	Point of A
//	final Point Point_A 	= new Point(10.015, -9.8, 4.493);	//	Point of A
//	final Point Point_A 	= new Point(11.21-0.78, -9.8, 4.79+0.1);	//	Point of A
	final Point Point_A 	= new Point(10.2216, -9.7889, 4.2917);	//	Point of A
	Point Point_A_Shift 	= new Point(0, 0, 0);					//	Point of A shift to target point.
	final Point Point_B 	= new Point(10.6, -8.0, 4.5);			//	Point of B
	Point Point_A_Prime 	= new Point(0, 0, 0);					//	Point of A prime
	Point Point_Target 		= new Point(11.21, -10.585, 5.375);	//	Default Point of Target.

	// 9.815, -9.806, 4.293
	protected void runPlan1()
	/* The winner's secret is here! */ {
		/* -> */	Log.i("Robot[Stage_1]: ", "Preparing to start");
		getQuaTargetPoint();
		QR_Discover();
		QRCodeFinish = false;
		// delay(2000);
		/* -> */	Log.i("Robot[Stage_2]: ", "Starting the mission");
		api.startMission();
//		long timeStart = SystemClock.elapsedRealtime();
//		moveTo(Point_A, Quaternion_Target, 3);
//		logMat(timeStart);
		QR_Discover();
//		for(int i=0; i<10; i++)
//		{
//			Kinematics robot = api.getRobotKinematics();
//			Point point = robot.getPosition();
//			Log.d("undock_point[" + i + "]", point.getX() + ", " +  point.getY() + ", " + point.getZ());
//		}
		//		/* -> */	Log.i("Robot[Stage_3]: ", "Laser is on");
//		api.laserControl(true);
//		/* -> */	Log.i("Robot[Stage_4]: ", "QR Code scanning");
//		QR_Discover();
//		/* -> */	Log.i("Robot[Stage_5]: ", "Get the target point");
//		getTargetPosition();
//		/* -> */	Log.i("Robot[Stage_6]: ", "Aim to target point");
//		moveTo(Point_A, Quaternion_Target, 3);
//		/* -> */	Log.i("Robot[Stage_7]: ", "Take a snapshot");
//		delay(2000);	api.takeSnapshot();
//		/* -> */	Log.i("Robot[Stage_8]: ", "Move to B point");
//		moveTo(Point_B, Zero_Quaternion, 2);
		/* -> */	Log.i("Robot[Stage_9]: ", "Mission completed");
		finishMission();
	}
	protected void runPlan2()
		/* Plan 2 */ {
	}
	protected void runPlan3()
		/* Plan 3 */ {
	}
	public double[] getVector(double[] p, double[] q)
		/* Finding a vector between of position (input two position array). */ {
		return new double[]{p[0]-q[0], p[1]-q[1], p[2]-q[2]};
	}
	public double[] getRoot(double a, double b, double c)
		/* Get the root of 2 degree equation (input coefficient a, b, c). */ {
		double InSqrt = Math.pow(b, 2)-4*a*c;
		return new double[]
				{
						(-b+Math.sqrt(InSqrt))/2*a,
						(-b-Math.sqrt(InSqrt))/2*a
				};
	}
	public double getVal(double[] p)
		/* Get the value of vector (input array of vector). */ {
		return Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2]));
	}
	public double[] getQua(double[] p, double[] q)
		/* Convert 2 vector to quaternion (input two array of vector). */{
		double [] n = vecCross(p,q);
		double [] r = {p[0]-q[0],p[1]-q[1],p[2]-q[2]};
		double [] v =
				{
						Math.sqrt((p[0]*p[0])+(p[1]*p[1])+(p[2]*p[2])),
						Math.sqrt((q[0]*q[0])+(q[1]*q[1])+(q[2]*q[2])),
						Math.sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2])),
						Math.sqrt((n[0]*n[0])+(n[1]*n[1])+(n[2]*n[2]))
				};
		double [] u = {n[0]/v[3], n[1]/v[3], n[2]/v[3]};
		double theta =  Math.acos(((v[0]*v[0])+(v[1]*v[1])-(v[2]*v[2]))/(2*v[0]*v[1]));
		return new double[]
				{
						Math.cos(theta/2),
						Math.sin(theta/2)*u[0],
						Math.sin(theta/2)*u[1],
						Math.sin(theta/2)*u[2]
				};
	}
	public double[] vecCross(double[] p, double[] q)
		/* Cross product of 3D vector (input 2 array of vector). */ {
		return new double[]
				{
						p[1]*q[2]-p[2]*q[1],
						p[2]*q[0]-p[0]*q[2],
						p[0]*q[1]-p[1]*q[0]
				};
	}
	public void delay(long sleep)
		/* sleep function */ {
		try
		{
			Thread.sleep(sleep);
		}
		catch (InterruptedException e)
		{
			e.printStackTrace();
		}
	}
	public void moveTo(Point point, Quaternion quaternion, int max_count)
		/* Re-check api.moveTo function is successful or until max number of counter. */ {
		Result result;
		int count = 0;
		do
		{
			result = api.moveTo(point, quaternion, false);
			count++;
		}
		while (!result.hasSucceeded() && count < max_count);
	}
	public void finishMission()
		/* Re-check api.reportMissionCompletion, The cause is sometimes unresponsive.*/ {
		Log.d("reportMissionCompletion[State]: ", "Starting");
		boolean ReportFinish = false;
		int loopCount = 0;
		while(!ReportFinish)
		{
			try
			{
				ReportFinish = api.reportMissionCompletion();
			}
			catch (Exception error)
			{
				Log.e("reportMissionCompletion[State]: ", "Failure");
				Log.e("reportMissionCompletion[Error]: ", error.getMessage());
			}
			Log.d("reportMissionCompletion[Count]: ", "" + loopCount);
			loopCount++;
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
	public Rect CustomCrop(int originWidth, int originHeight, int offsetTop, int offsetBottom, int offsetLeft, int offsetRight)
		/* Calculate crop image, requiring both sides offset of x and y. */ {
		Log.d("CustomCrop[State]: ", "Starting");
		int width	= originWidth-offsetLeft-offsetRight;		// Calculate for width of image
		int height 	= originHeight-offsetTop-offsetBottom;		// Calculate for height of image
		Log.d("CustomCrop[State]: ", "Finished");
		return new Rect(offsetLeft, offsetTop, width, height);
	}
	public Mat QR_Discover()
		/* All events that take position at the point of A include QR Code Reading, AR Code Reading and Target position analyzing. */ {
		/* Set parameter related to image from navigation camera. */
//		Rect crop 	= CustomCrop(1280, 960, 60, 450, 400, 430);
		Rect crop 	= CustomCrop(1280, 960, 510, 0, 80, 750);
		Rect raw 	= CustomCrop(1280, 960,0,0,0,0);
		/* Set parameter about QR Code. */
		Mat	matSrc 	= new Mat(),		// Matrix for storing image form navigation camera.
				matSrc_raw = new Mat(1280, 960, CvType.CV_8UC1);
		Bitmap bitmapSrc;				// Bitmap image according to the requirement of QR reading.
		String QR_Info = null; 			// Content from QR Code reader.
		/* Set general parameter */
		int loopCount = 0; 				// Variable of loop counter.
		final int loopCountDetect = 99;	// Variable of max loop count.	//	99 = Disable
		final int loopCountMax = 6;		// Variable of max loop count.

		while(!QRCodeFinish && loopCount < loopCountMax)
		{ /* The condition is defined with success ar code reading and the number of loops (the AR code must be read before the AR code can be read). */
			Log.d("Move&GetImage[State]:", " Starting");
			long timeStart = SystemClock.elapsedRealtime();

			// First step : Movement to point A positionn -> Get image form Nav Camera -> Get realtime position of robot.
//			if(loopCount%2 == 0)
			if(loopCount == 0 && FirstRun)
			{
				Imgproc.resize(getQRTest(), matSrc, new Size(200,200));
				bitmapSrc = Bitmap.createBitmap(matSrc.width(), matSrc.height(), Bitmap.Config.ARGB_8888);
				QRCodeFinish = true;
				Log.d("DeadFunc","*************************************************************");
			}
			else
			{
				moveTo(Point_A, Zero_Quaternion, 3);
//				moveTo(Point_A, Quaternion_Target, 3);
				matSrc_raw = api.getMatNavCam();	//	Get image form navigation camera.

				if(loopCount >= loopCountDetect)	// 	Cope with situations where the robot loses self-localization.
				{
					matSrc = new Mat(matSrc_raw, raw);
					bitmapSrc = Bitmap.createBitmap(raw.width, raw.height, Bitmap.Config.ARGB_8888);
				}
				else
				{
//						matSrc = new Mat(matSrc_raw, crop);

					Imgproc.warpPerspective(new Mat(matSrc_raw, crop), matSrc, getH(), new Size(crop.width, crop.height));
					Imgproc.resize(matSrc, matSrc, new Size(900,900));
					//					bitmapSrc = Bitmap.createBitmap(crop.width, crop.height, Bitmap.Config.ARGB_8888);
					bitmapSrc = Bitmap.createBitmap(matSrc.width(), matSrc.height(), Bitmap.Config.ARGB_8888);
				}
			}
			Log.d("Move&GetImage[State]:", " Finished");
			Log.d("Move&GetImage[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);

			// Second step (QR code event) : Get image form Nav Camera -> QR Code decoder -> Send data of QR code to judge.
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
					if(!FirstRun)	api.sendDiscoveredQR(QR_Info); // Send data of QR code to judge.

					double[] Buf = StringParseToDouble(QR_Info);
					Pattern = (int) Buf[0];
					Point_A_Prime = new Point(Buf[1], Buf[2], Buf[3]);
					Log.d("QR[Data][Pattern]:", " " + Pattern);
					Log.d("QR[Data][A_Prime]:", " X: " + Point_A_Prime.getX() + ", Y: " + Point_A_Prime.getY() + ", Z: " + Point_A_Prime.getZ());
					QRCodeFinish = true; // Set new condition.
				}
				Log.d("QR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart) / 1000);
			}
			Log.d("A_Event[State]:", " Finished");
			Log.d("A_Event[Count]:", " " + loopCount);
			FirstRun = false;
			loopCount++;
		}
		return matSrc_raw;
	}
	public void getQuaTargetPoint()
		/* Similar to the moveTo six variables but between is vector ends are pointed by a laser pointer not center of robot. */ {
		double [] robot_pos		= {Point_A.getX(), Point_A.getY(), Point_A.getZ()};
		double [] target_pos	= {Point_Target.getX(), Point_Target.getY(), Point_Target.getZ()};
		double [] target_vec	= getVector(target_pos, robot_pos);
		double target_val		= getVal(target_pos);
		double [] laser_shift	= {0.1302, 0.0572, -0.111} ;
		double laser_val		= getVal (laser_shift);
		double las_angle		= 136.11699 * 0.01745;
		double [] length		= getRoot(1,-2*laser_val*Math.cos(las_angle),(laser_val*laser_val)-(target_val*target_val));
		double laser_length;
		if (length[0] > 0 )		laser_length = length[0];
		else 					laser_length = length[1];

		double [] origin_vec	= {laser_shift[0]+laser_length, laser_shift[1], laser_shift[2]};
		double [] Qua			= getQua(origin_vec, target_vec);
		Quaternion_Target = new Quaternion((float) Qua[1], (float) Qua[2], (float) Qua[3], (float) Qua[0]);
		Log.d("FinalQuaternion= ", "w: " + Qua[0] + ", a: " + Qua[1] + ", b: " + Qua[2] +", c: " + Qua[3]);
	}
	public double[] SrcPointTransformToHomoPoint(Mat H, double[] pt)
		/* Position transformation form source image to homography image. */ {
		/* Input ; 3x3 Mat -> Homography constant, 2 double in array -> point x and point y */
		Mat pt1 = new Mat(3, 1, CvType.CV_64FC1);
		Mat	pt2 = new Mat();
		pt1.put(0, 0, pt[0], pt[1], 1);

		Core.gemm(H, pt1, 1, new Mat(), 0, pt2);

		double[] data = pt2.get(2, 0);
		Core.divide(pt2, new Scalar(data[0]), pt2);

		double[] data1 = pt2.get(0, 0);
		double[] data2 = pt2.get(1, 0);
		return new double[]{data1[0], data2[0]};
	}
	public void getTargetPosition(Mat matSrc_raw)
		/* Compute the coordinates of the target position transformed by the homography. */ {
		/* Set AR parameter */
		Mat	matSrc;									// Matrix for storing image form navigation camera.
		Mat IDs = new Mat(); 						// Variable store ID each of AR code. : Matrix format
		List<Mat> Corners = new ArrayList<>(); 	// Variable store four corner each of AR Code.
		Dictionary Dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250); // Standard AR code format according rule book.
		int[] AR_ID = new int[]{0, 0, 0, 0};		// Variable store ID each of AR code. : Array of Int format
		/* Set general parameter */
		Rect crop 	= CustomCrop(1280, 960, 235, 275, 500, 330);
		Rect raw 	= CustomCrop(1280, 960,0,0,0,0);
		int loopCount = 0; 				// Variable of loop counter.
		final int loopCountDetect = 2;	// Variable of max loop count.
		final int loopCountMax = 6;		// Variable of max loop count.
		/* Target tag variables */
		int[] 	ArID1_CornerRT = new int[]{0, 0},	// Container the specified corner of the ar tag.
				ArID2_CornerLT = new int[]{0, 0},
				ArID3_CornerLB = new int[]{0, 0},
				ArID4_CornerRB = new int[]{0, 0};
		int virtualWidth	= 275,	//	Virtual width of AR tag. 	//	Reference ratio from the real tag.
				virtualHeight 	= 133;	//	Virtual height of AR tag.	//	in meter unit.
		int offsetX = 0,			// 	Origin shifting for warp process.
				offsetY = 0;			//	..
		MatOfPoint2f 	corners1 = new MatOfPoint2f(),	//	Use contain input corners of the image A.
				corners2 = new MatOfPoint2f();	//	Use contain output c00.orners of the image B.
		Mat H;											// Constant variables of Homography.
		double[] laserCenterSrc;						// Center of laser pointer on image. (Constant point)

		while(!ARCodeFinish && loopCount < loopCountMax)
		{
			Log.d("AR[State]:", " Starting");
			Log.d("Move&GetImage[State]:", " Starting");
			long timeStartAR = SystemClock.elapsedRealtime();		// Start timer
			if(loopCount%2 == 0)
				moveTo(Point_A, Quaternion_Target, 5);	// Rotate to target point again.
			matSrc_raw = api.getMatNavCam();	//	Get image form navigation camera.
			if(loopCount >= loopCountDetect)	// 	Cope with situations where the robot loses self-localization.
			{
				matSrc = new Mat(matSrc_raw, raw);
				laserCenterSrc = new double[]{693, 464};	//	Laser pointer on raw image.
			}
			else
			{
				matSrc = new Mat(matSrc_raw, crop);
				laserCenterSrc = new double[]{203, 228};	//	Laser pointer on crop image.
			}
			Log.d("Move&GetImage[State]:", " Finished");
			Log.d("Move&GetImage[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStartAR) / 1000);

			try
			{
				Aruco.detectMarkers(matSrc, Dict, Corners, IDs); 	// AR tag detector
				AR_ID = new int[] 									// Put ID of AR Code to Array of int.
						{
								(int) IDs.get(0, 0)[0],
								(int) IDs.get(1, 0)[0],
								(int) IDs.get(2, 0)[0],
								(int) IDs.get(3, 0)[0]
						};
				Log.d("AR[State]:", " Detected");
				Log.d("AR[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStartAR) / 1000);
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
								{						// Return variables sorted clockwise starting from left-top corner.
										{(int) Corners.get(i).get(0, 0)[0], (int) Corners.get(i).get(0, 0)[1]},	// Left-top
										{(int) Corners.get(i).get(0, 2)[0], (int) Corners.get(i).get(0, 2)[1]},	// Right-bottom
										{(int) Corners.get(i).get(0, 1)[0], (int) Corners.get(i).get(0, 1)[1]},	// Right-top
										{(int) Corners.get(i).get(0, 3)[0], (int) Corners.get(i).get(0, 3)[1]}	// Left-bottom
								};
						for(int j=0; j<4; j++)	// Log corner of ar tag.
							Log.d("AR_Corner[" + i + "][" + j + "]:", " X= " + Corner[j][0] + ", Y= " + Corner[j][1]);

						/* Set the corner to match the id of ar code. */
						if (AR_ID[i] == 1){ ArID1_CornerRT[0] = (int) Corner[2][0]; ArID1_CornerRT[1] = (int) Corner[2][1]; }
						else if (AR_ID[i] == 2){ ArID2_CornerLT[0] = (int) Corner[0][0]; ArID2_CornerLT[1] = (int) Corner[0][1]; }
						else if (AR_ID[i] == 3){ ArID3_CornerLB[0] = (int) Corner[3][0]; ArID3_CornerLB[1] = (int) Corner[3][1]; }
						else if (AR_ID[i] == 4){ ArID4_CornerRB[0] = (int) Corner[1][0]; ArID4_CornerRB[1] = (int) Corner[1][1]; }
					}
					org.opencv.core.Point[] cornerIN =		// Set input corners with four AR tag.
							{
									new org.opencv.core.Point(ArID1_CornerRT[0], ArID1_CornerRT[1]),
									new org.opencv.core.Point(ArID2_CornerLT[0], ArID2_CornerLT[1]),
									new org.opencv.core.Point(ArID3_CornerLB[0], ArID3_CornerLB[1]),
									new org.opencv.core.Point(ArID4_CornerRB[0], ArID4_CornerRB[1])
							};
					/* Data logger of each ar tag corner */
					Log.d("ARCorner(ID1)", " X= " + ArID1_CornerRT[0] + ", Y= " + ArID1_CornerRT[1]);
					Log.d("ARCorner(ID2)", " X= " + ArID2_CornerLT[0] + ", Y= " + ArID2_CornerLT[1]);
					Log.d("ARCorner(ID3)", " X= " + ArID3_CornerLB[0] + ", Y= " + ArID3_CornerLB[1]);
					Log.d("ARCorner(ID4)", " X= " + ArID4_CornerRB[0] + ", Y= " + ArID4_CornerRB[1]);

					org.opencv.core.Point[] cornerOut =		// Set output corners with actual ratio of AR tag.
							{
									new org.opencv.core.Point(virtualWidth + offsetX, offsetY),
									new org.opencv.core.Point(offsetX, offsetY),
									new org.opencv.core.Point(offsetX,virtualHeight + offsetY),
									new org.opencv.core.Point(virtualWidth + offsetX,virtualHeight + offsetY)
							};
					corners1.fromArray(cornerIN);	// input array into matrix of point variable.
					corners2.fromArray(cornerOut);	// ..
					H = Calib3d.findHomography(corners1, corners2);	// Calculate constant variables of Homography.
					Log.d("Homography=", " " + H.dump());

					double[] TargetCenterHomo = {offsetX+(virtualWidth/2f), offsetY+(virtualHeight/2f)};	// Center of Tatget tag on homo image. (Constant point)
					double[] laserCenterHomo = SrcPointTransformToHomoPoint(H, laserCenterSrc);			// Get laser position on homography image.
					double X_dif = (laserCenterHomo[0] - TargetCenterHomo[0]);		// The x-axis distance between the robot and the target : meter unit
					double Y_dif = (laserCenterHomo[1] - TargetCenterHomo[1]);		// The y-axis distance between the robot and the target : meter unit
					Log.d("TargetCal[Data][Dif]:", " X: " + X_dif + ", Y: " + Y_dif);			// Logger of raw x and y dif
//					X_dif /= solveLinear(Coeffs_X.x, Coeffs_X.y, X_dif/1000);
//					Y_dif /= solveLinear(Coeffs_Y.x, Coeffs_Y.y, Y_dif/1000);
					X_dif /= 1000;
					Y_dif /= 1000;
					Log.d("TargetCal[Data][Dif]:", " X: " + X_dif + ", Y: " + Y_dif);			// Logger of process x and y dif

					Point_Target = new Point(	Point_Target.getX() - X_dif,
							Point_Target.getY() + 0,
							Point_Target.getZ() - Y_dif);	// Calculate target position on ISS.
					Point_A_Shift = new Point(	Point_A.getX() - 0,
							Point_A.getY() + 0,
							Point_A.getZ() + 0);		// Calculate new robot position when laser pointer target point with same quaternion.
					getQuaTargetPoint();	//	Quaternion update by new target point.

					/* Data logger */
//					Log.d("TargetCal[Data][Dif]:", " X: " + X_dif + ", Y: " + Y_dif);
					Log.d("TargetCal[Data][P_T-Point]:", " X: " + Point_Target.getX() + ", Y: " + Point_Target.getY() + ", Z: " + Point_Target.getZ());
					Log.d("TargetCal[Data][P_A-Point]:", " X: " + Point_A_Shift.getX() + ", Y: " + Point_A_Shift.getY() + ", Z: " + Point_A_Shift.getZ());
					Log.d("TargetCal[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStart2));
					ARCodeFinish = true; // Set new condition.
				}
				Log.d("AR+Cal[Total_Time]:", " " + (SystemClock.elapsedRealtime() - timeStartAR) / 1000);
			}
			Log.d("AR[Count]:", " " + loopCount);
			loopCount++;
		}
	}
	public void getTargetPosition()
		/*  */ {
		if(Pattern == 1)	Point_Target = new Point(11.0947 + 0.01629 + 0.00229,-10.585,5.4221 + 0.02381 - 0.00455);
		else if(Pattern == 2)	Point_Target = new Point(11.0592 + 0.01439 - 0.00121,-10.585,5.3730 + 0.02637 - 0.00308);
		else if(Pattern == 3)	Point_Target = new Point(11.0386 + 0.00726 + 0.00053,-10.585,5.4092 + 0.02383 - 0.00310);
		else if(Pattern == 4)	Point_Target = new Point(11.1004 + 0.01771 - 0.00059,-10.585,5.3993 + 0.02521 + 0.00033);
		else if(Pattern == 5)	Point_Target = new Point(11.0431 + 0.02489 - 0.01061,-10.585,5.3570 + 0.02558 - 0.00150);
		else if(Pattern == 6)	Point_Target = new Point(11.0976 + 0.01925 - 0.00236,-10.585,5.4233 + 0.01907 + 0.00355);
		else if(Pattern == 7)	Point_Target = new Point(11.0438 + 0.01106 - 0.00368,-10.585,5.3964 + 0.01967 + 0.00171);
		else if(Pattern == 8)	Point_Target = new Point(11.0545 + 0.01605 + 0.00058,-10.585,5.3689 + 0.01884 - 0.00137);

		getQuaTargetPoint();
	}
	public Mat getH()
		/*  */ {
//		final double[] H_Array =
//		{
//			0.6347369030483551,		-0.2865184064434098,	83.37897709665451,
//			0.1244997438688915,		0.7576878728075858, 	-36.09958001446229,
//			-0.001363118281535927,	-1.393442149674069e-05,	1
//		};
		final double[] H_Array =
				{
						0.900417921032328,		-0.3502817876735206,	40.18939928174051,
						0.5441117400558353,		0.8458889627475615,		-279.6971999106549,
						-0.001735209403836954,	-0.0001491801126560375,	1
				};
		Mat H = new Mat(3, 3, CvType.CV_32F);
		H.put(0, 0, H_Array);
		return H;
	}
	public Mat getQRTest()
		/*  */ {
		final double[] array =
				{
						126,  65,  64,  63,  60,  61,  59,  59,  57,  58,  54, 105, 232, 190,  59,  46,  42,  39, 116, 233, 139,  68, 189, 189,  66, 114, 238, 255, 224,  99, 116, 236, 254, 245, 117,  90, 243, 209,  71,  50,  55,  55,  61,  62,  63,  67,  65,  69,  68, 102,
						83,  12,  62,  66,  70,  72,  74,  76,  76,  70,  26,  69, 225, 167,  12,   0,   0,   2,  94, 235, 165, 104, 206, 199, 100, 142, 234, 201, 163, 118, 103, 167, 182, 174, 117, 118, 232, 197,  31,  38,  74,  79,  79,  80,  79,  78,  78,  65,  31,  53,
						79,  51, 218, 238, 237, 235, 238, 236, 236, 221,  80,  67, 226, 176,  20,   5,   5,   3,  87, 232, 249, 243, 246, 251, 245, 245, 239, 111,  68, 187, 168,  46,  33,  43, 166, 229, 251, 195,  42, 110, 223, 232, 227, 229, 223, 221, 221, 200,  91,  56,
						86,  48, 234, 206, 157, 154, 149, 150, 202, 235,  85,  68, 223, 168,  24,  84, 107,  32,  51, 136, 211, 240, 172, 190, 249, 211, 155, 138, 124, 124, 103,  17,  83, 123, 143, 160, 239, 193,  40, 130, 235, 182, 156, 153, 154, 162, 210, 227, 104,  55,
						83,  53, 224, 146,  14,   4,   5,  11, 104, 226,  87,  71, 226, 166,  35, 164, 217,  84,   7,  25, 149, 235,  97, 100, 227, 173,  61, 178, 208,  69,  16,  25, 157, 227,  99,  64, 240, 192,  45, 128, 220,  97,  21,  17,  21,  32, 133, 214,  99,  63,
						88,  54, 228, 138,   8,   0,   0,   5, 109, 232,  87,  69, 224, 170,  23,  51, 101, 154, 114,  27, 150, 243, 198, 142, 101, 135, 171, 226, 244, 193, 167, 163, 111, 102, 136, 186, 247, 191,  40, 130, 224,  78,   5,   1,   0,  17, 127, 214, 104,  54,
						89,  53, 220, 143,  11,   0,   0,   6, 106, 224,  84,  74, 227, 185,  53,  41,  66, 196, 186,  78, 163, 245, 218, 149,  20,  73, 199, 215, 214, 219, 219, 209,  96,  58, 152, 233, 252, 191,  46, 137, 214,  80,   3,   0,   1,  19, 126, 211,  95,  65,
						87,  58, 226, 142,   9,   0,   0,   5, 106, 228,  90,  67, 223, 245, 223, 220, 223, 246, 246, 226, 237, 233, 104,  35,   5,  15,  51,  55,  56,  58,  56,  65, 150, 195, 114, 108, 234, 189,  40, 131, 224,  77,   7,   1,   1,  19, 129, 211, 102,  59,
						91,  52, 223, 172,  72,  61,  61,  66, 139, 225,  80,  75, 226, 232, 209, 241, 255, 255, 235, 208, 211, 198,  69,   2,   0,  20,  56,  19,   1,   1,  19,  61, 166, 192,  75,  50, 229, 188,  46, 144, 223, 110,  48,  47,  48,  61, 152, 215,  91,  65,
						88,  54, 225, 246, 233, 230, 234, 233, 239, 236,  89,  69, 221, 179,  57, 183, 254, 250, 174,  48,  23,  19,  11,   2,   1,  91, 210,  59,   2,   3,  64, 205, 114,  42,  16,  63, 232, 185,  42, 126, 245, 229, 212, 212, 207, 208, 228, 223,  98,  66,
						88,  18, 107, 131, 128, 126, 126, 127, 126, 117,  42,  74, 227, 159,  38, 178, 237, 159, 147, 126,  53,  13, 100,  77,   6,  91, 223,  84,  28, 104, 121, 147, 127, 108,  40,  56, 226, 185,  35,  90, 156, 166, 167, 172, 174, 172, 173, 156,  72,  64,
						101,  14,  14,  17,  15,  14,  15,  19,  19,  18,  16,  80, 223, 175,  49, 168, 221,  87, 109, 228, 108,  38, 190, 152,  15,  98, 234,  79,  55, 199, 181,  44, 157, 218,  85,  71, 238, 185,  37,  22,  26,  32,  31,  29,  33,  29,  29,  28,  20,  72,
						203, 164, 167, 166, 169, 167, 168, 172, 171, 170, 171, 190, 245, 228, 164, 119, 112, 161, 195, 235, 112,  30, 193, 170,  15,  94, 234, 184, 141, 124, 139, 137, 124, 126, 139, 165, 239, 224, 137, 125, 123, 121, 119, 115, 111, 108, 104, 100,  98, 130,
						231, 220, 221, 241, 254, 255, 247, 225, 228, 229, 224, 241, 253, 255, 244, 118,  64, 199, 241, 218, 119,  72, 178, 132,  15,  97, 243, 237, 190,  45,  76, 209, 102,  57, 194, 251, 255, 255, 247, 237, 235, 242, 240, 229, 222, 228, 225, 227, 230, 234,
						112,  42,  46, 145, 248, 254, 202,  71,  40,  36,  40, 104, 234, 254, 254, 234, 223, 241, 169,  75, 153, 200,  90,  39,   6,  98, 229, 129,  56,  15,  18,  79, 170, 195, 238, 255, 255, 255, 242, 159, 125, 222, 217, 133, 105, 108, 112, 113, 122, 162,
						91,   2,   3,  78, 166, 174, 162, 104,  49,  11,  66, 103, 161, 182, 183, 184, 184, 184, 155,  99, 135, 172, 103,  81,  80, 110, 180, 112,  58,  14,  20,  83, 199, 252, 219, 207, 208, 212, 204, 128,  82, 168, 168,  46,   9,  15,  36,  41,  26,  83,
						93,   3,   1,   7,  17,  24,  75, 212, 141,  34, 162, 183,  46,  22,  22,  23,  23,  25, 119, 220, 107,  45, 193, 232, 230, 162,  63, 171, 181,  37,  76, 219, 245, 236, 122,  50,  48,  50,  66, 148, 180,  86,  49,  11,   0,  20, 108, 143,  61,  75,
						91,   1,   0,   0,   6,  91, 122, 133, 146, 124, 130, 109,  23,   0,  10,  85, 109, 107, 119, 150,  70,  24, 127, 164, 164, 114,  19, 123, 138,  30,  94, 242, 255, 240, 134,  57,   5,   3,  27, 161, 219, 106,  54,  42,  20,  22, 113, 170,  72,  75,
						94,   4,   0,   0,  10, 160, 200,  41, 114, 225, 110,  34,  28,  18,  24, 182, 245, 237, 138,  18,  17,  16,   5,  14,  25,  11,   3,  23,  18,   7,  87, 254, 250, 246, 241, 170,  36,   9,  23, 139, 243, 237, 229, 215, 120,  22,  19,  36,  24,  80,
						91,   2,   0,   0,  11, 169, 195,  40, 113, 243, 216, 192, 188, 118,  27, 185, 243, 129,  55,  13, 108, 147,  37,  77, 162, 103,  14, 129, 132,  27, 101, 231, 165, 140, 224, 227, 143,  83,  23,  84, 166, 237, 255, 249, 188, 103,  37,   4,   8,  79,
						94,   4,   0,  20,  40, 169, 212,  67, 110, 219, 229, 236, 253, 166,  38, 182, 226,  59,  17,  42, 154, 213,  58,  95, 248, 157,  59, 194, 190,  40,  76, 228,  87,  44, 182, 247, 241, 166,  33,  18,  69, 210, 255, 255, 250, 215, 105,  22,  18,  86,
						93,   2,   4, 108, 213, 241, 246, 225, 135,  56,  51, 116, 238, 167,  32, 185, 232,  66,  71, 185, 119,  72,  14, 118, 246, 236, 180, 101,  73,  11,  38,  84,  27,  12,  68, 153, 246, 221, 131,  47,  56, 209, 255, 255, 255, 246, 180, 126, 117, 152,
						174, 114, 104, 170, 248, 255, 227, 171, 145, 110, 102, 137, 238, 165,  31, 119, 165, 103, 166, 241, 153,  76,  16,  68, 189, 198, 181,  86,  57,  56,  53,  49,  43,  33,  13,  89, 247, 240, 203, 111,  75, 189, 227, 233, 247, 251, 235, 219, 221, 234,
						254, 253, 248, 251, 255, 253, 201,  46, 115, 238, 247, 247, 254, 163,  13,  10,  40, 190, 245, 255, 245, 211,  55,  16,  33,  29,  46, 179, 228, 230, 227, 228, 228, 194,  71,  87, 230, 179,  82, 158, 171,  87,  72,  96, 181, 228, 151,  98, 102, 159,
						255, 230, 139, 195, 253, 168, 144, 144, 120, 133, 140, 172, 248, 166,  12,   0,  17, 119, 201, 253, 255, 243, 144, 102,  99,  55,  25, 146, 180, 185, 213, 255, 255, 222,  67,  86, 246, 181,  86, 149, 166,  45,  25,  59, 139, 189, 117,  44,  21,  95,
						255, 226,  65, 130, 237,  99,  64, 211, 142,  14,  13,  89, 226, 161,  11,   0,   1,   9, 103, 250, 251, 251, 244, 237, 233, 152,  17,  32,  32,  32, 129, 250, 255, 224,  84,  99, 235, 245, 203, 109,  41,  17,  73, 171, 123,  96, 151, 137,  49,  95,
						255, 252, 212, 226, 251,  92,  10,  59,  43,   8, 141, 151,  88,  48,   5,   0,   0,   0, 120, 238, 161, 115, 231, 208, 115, 128, 143, 141, 106,  23, 106, 249, 255, 241, 155, 115, 161, 211, 223,  84,   6,   9,  86, 215, 164,  99, 136, 161, 114, 138,
						211, 183, 185, 224, 247, 101,  17,  63,  66,  74, 168, 160,  77,  29,   2,   0,   4,  39, 104, 203, 117,  71, 199, 125,  15,  90, 222, 246, 190,  36,  95, 234, 252, 255, 244, 163,  65, 135, 220,  98,  12,  17,  94, 232, 244, 198, 109,  95, 192, 226,
						107,  27,  27, 130, 246,  90,  60, 207, 231, 228,  86,  90, 210, 146,   7,   1,  34, 187, 136,  65, 159, 203,  88,  31,   2,  27,  94, 232, 196,  41,  42, 111, 216, 255, 255, 227, 175, 202, 226,  87,  46, 125, 176, 249, 255, 245, 180, 159, 230, 254,
						98,   4,   1, 118, 240, 167, 113, 166, 177, 169, 107, 134, 240, 157,  17,  51,  84, 216, 181,  75, 158, 205,  69,  40,  36,  22,  42, 205, 191,  34,   4,  37, 197, 254, 254, 247, 234, 235, 216,  97,  78, 203, 244, 253, 250, 251, 251, 247, 250, 255,
						116,  26,  28, 126, 248, 243, 176,  37,  20,  39, 182, 233, 243, 158,  33, 167, 229, 244, 248, 217,  98,  47, 194, 224, 229, 115,  36, 213, 196,  36,   2,  44, 204, 255, 253, 196,  89,  84, 107, 151, 141, 117, 171, 229, 174, 157, 222, 212, 164, 197,
						226, 201, 200, 225, 250, 147,  62,  10,  72, 165, 129,  96,  99,  69,  11,  76, 111, 119, 123, 112,  44,  14, 114, 139, 144,  81,  46, 210, 186,  34,   2,  38, 197, 255, 250, 163,  10,   2,  29, 132, 142,  39, 105, 205, 101,  47, 138, 155,  61, 113,
						255, 255, 255, 255, 251,  95,   9,  32, 110, 222,  90,  32,  30,  20,   1,   3,   5,   6,   6,   8,  14,  15,   9,  16,  23,  11,  35, 213, 192,  31,   5,  48, 207, 253, 252, 158,  20,   2,   6,  29,  36,  24, 100, 215, 113,  24,  46,  56,  33, 108,
						255, 255, 255, 255, 251,  99,  44, 182, 154,  85, 152, 186, 184, 109,   5,   0,   0,   0,   0,  12, 104, 124,  21,  72, 138,  77,  50, 216, 184,  38,  52, 119, 144, 186, 245, 209,  92,  45,   4,   3,  29,  78, 148, 226, 143,  74,  66,  64,  73, 138,
						198, 161, 165, 209, 249,  98,  62, 229, 197,  86, 157, 190, 195, 143,  65,  57,  55,  52,  47,  55, 157, 184,  45, 128, 246, 152,  56, 197, 176,  30, 122, 203,  64,  69, 205, 244, 218, 116,   6,   5,  72, 185, 229, 247, 229, 208, 204, 200, 199, 219,
						110,  17,  19, 128, 248, 107,  62, 238, 251, 227,  98,  34,  30, 118, 226, 226, 224, 227, 222, 207, 108,  44,  14, 117, 253, 227, 171,  97,  55,  15,  37,  67,  18,  16,  72,  96, 100,  64,   4,   3,  35, 116, 137, 168, 230, 252, 251, 222, 189, 215,
						202, 155, 152, 198, 253, 185, 169, 244, 255, 253, 168, 119, 114, 140, 158, 166, 187, 246, 255, 250, 147,  69,  18, 107, 194, 203, 180,  94,  58,  50,  24,   1,   0,   0,   2,  20,  32,  33,  30,  25,  16,  12,  25,  71, 190, 249, 240, 174,  76, 138,
						248, 241, 245, 244, 247, 242, 246, 251, 248, 250, 241, 246, 244, 158,  34,  30,  65, 213, 253, 254, 240, 207,  43,  25,  48,  48,  83, 188, 209, 202, 105,   6,   1,   0,   5,  84, 178, 180, 176, 167, 107,  26,  73, 159, 216, 226, 160, 100,  41, 121,
						140,  57,  59,  62,  70,  74,  76,  82,  89,  91, 100, 148, 246, 216, 159, 148, 136, 125, 197, 255, 255, 235, 137, 112, 103,  56,  37, 141, 208, 249, 181,  63,  14,   1,   7, 117, 251, 233, 229, 246, 175,  40, 118, 236, 252, 219, 100,  38,  30, 123,
						96,  16,  63,  73,  71,  68,  66,  59,  60,  53,  17,  80, 242, 237, 228, 247, 220,  58, 135, 252, 243, 238, 249, 235, 225, 109,  20,  29, 102, 228, 243, 182,  47,   6,   7, 133, 236, 159, 109, 208, 165,  45, 119, 245, 255, 243, 189, 106,  41, 129,
						109,  22, 219, 248, 243, 241, 240, 238, 235, 227,  56,  73, 241, 183,  76, 208, 229,  47, 115, 251, 136, 117, 232, 164,  97, 122, 132,  32,  41, 109, 192, 235, 159,  93,  26, 117, 249, 180, 133, 218, 174,  41, 123, 249, 255, 238, 203, 137,  42, 127,
						100,  40, 228, 226, 183, 189, 189, 196, 226, 241,  79,  80, 240, 159,  21, 164, 191,  42, 120, 208, 104,  62, 198, 127,   8, 139, 197,  38,   3,  14, 161, 250, 241, 179,  47, 139, 244, 247, 242, 245, 159,  42, 129, 242, 252, 205,  94,  37,  34, 136,
						108,  26, 229, 162,  17,   9,   7,  15, 113, 250,  71,  72, 241, 159,  12,  28,  30,  10,  23,  63, 162, 171,  82,  38,   5,  48,  64,  14,   2,  12, 134, 230, 150,  95,  30,  71, 145, 152, 161, 163, 109,  28,  82, 203, 240, 191,  63,  37,  73, 154,
						105,  39, 223, 158,   7,   0,   0,   2, 108, 234,  80,  83, 241, 228, 127,  28,   0,   0,  63, 113, 142, 144, 120, 100,  89,  35,  20,  68,  46,  15, 154, 209,  54,   5,   3,  32,  56,  56,  53,  55,  46,  31,  22,  65, 182, 186,  74,  95, 210, 235,
						109,  29, 230, 163,  13,   0,   0,   5, 107, 249,  74,  68, 242, 255, 230,  73,  15,  16, 108, 226,  82,  63, 212, 238, 229, 102,  57, 197, 141,  37, 149, 199,  43,   1,   8, 102, 192, 188, 183, 179, 172, 154,  80,  22,  84, 138, 139, 127, 149, 214,
						109,  40, 220, 159,  10,   2,   1,   4, 110, 232,  78,  86, 241, 255, 251, 196, 163, 161, 130, 114,  40,  29, 114, 144, 143,  73,  35, 135, 141, 108, 182, 206,  49,   2,  10, 138, 253, 252, 242, 218, 229, 240, 143,  35,  19,  74, 178, 139,  56, 153,
						110,  31, 231, 196,  99,  83,  76,  75, 147, 250,  73,  76, 243, 245, 230, 249, 255, 252, 151,  10,  18,  16,  13,  22,  30,  22,  12,  24, 108, 211, 245, 203,  42,   3,   9, 153, 249, 252, 210, 117, 167, 245, 214, 137,  42,  27,  81,  68,  41, 149,
						111,  38, 207, 245, 236, 239, 236, 241, 243, 233,  74,  86, 240, 177,  54, 211, 254, 255, 118,  33, 150, 156,  35, 115, 178, 175, 135,  28,  45, 111, 204, 227, 149,  83,  20,  83, 163, 168, 138,  53, 122, 243, 220, 170,  95,  37,  15,  11,  39, 151,
						113,  12,  85, 107, 113, 118, 123, 126, 134, 133,  44,  76, 244, 139,  27, 142, 192, 249, 179,  90, 163, 170,  35, 138, 254, 254, 217,  71,  23,  14, 151, 227, 226, 154,  30,   9,  20,  34,  41,  34, 131, 235, 144, 106, 183, 144,  33,   9,  43, 156,
						137,  40,  38,  38,  36,  33,  33,  29,  30,  29,  21,  99, 240, 168,  15,  25,  83, 237, 248, 219,  91,  32,  11, 171, 254, 255, 247, 212, 115,  18,  48,  86,  95,  66,  17,   2,  18,  97, 140, 136, 194, 241, 187, 151, 224, 176,  52,  50, 106, 186
				};
		Mat out = new Mat(50, 50, CvType.CV_8U);
		out.put(0, 0, array);
		return out;
	}
	public void logMat(long timeStart)
	{
		Mat src = new Mat();
		src.setTo(new Scalar(255, 255, 255));
		src = api.getMatNavCam();
		int rows = 135,
				cols = 181;
		Imgproc.resize(src, src, new Size(cols, rows));
		for (int r=0; r<rows; r++)
		{
			for(int c=0; c<cols; c++)
			{
				double data[] = src.get(r, c);
				Log.d(r + "," + c, "" + (int)data[0]);
				if(SystemClock.elapsedRealtime() - timeStart >= 288000)	break;
			}
		}
	}
}
