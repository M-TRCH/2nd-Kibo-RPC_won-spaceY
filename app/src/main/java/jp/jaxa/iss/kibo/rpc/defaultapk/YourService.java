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
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

public class YourService extends KiboRpcService
{
	// Score : {65.14, 72.0, 73.06, 64.08, 68.10, 70.61, 68.20, 66.36, 72.32, 66.29}
	@Override
	protected void runPlan1()
	{
		api.startMission();
		Log.d("Robot[state]:", "Start mission");

		Log.d("Robot[state]:", "Start QR");
		final double[] A2 = StringParseToDouble(QR_event_fast(11.21f, -9.8f, 4.79f, 0f, 0f, -0.707f, 0.707f, 5));

		Log.d("Robot[state]:", "Start AR");
		double PixelToMeter = 533.333333333;
		double[] target = AR_event(5);
		Log.d("AR[CenX]: ", "" + target[0]);
		Log.d("AR[CenY]: ", "" + target[1]);

		double dis_x = 640-target[0]; // - -> right
		double dis_y = 480-target[1]; // + -> top
		Log.d("AR[DisX]: ", "" + dis_x);
		Log.d("AR[DisY]: ", "" + dis_y);

		dis_x /= PixelToMeter;
		dis_y /= PixelToMeter;
		Log.d("AR[DisX]*: ", "" + dis_x);
		Log.d("AR[DisY]*: ", "" + dis_y);

		double x_shift  = 0.1825;
		double y_shift  = 0.1415;
		double[] Target = new double[]{11.21f-dis_x, -10.585, 4.79f-dis_y};

		Log.d("Robot[state]:", "Start rotate");
		moveTo(11.21f, -9.8f, 4.79f, Target[0]-x_shift, Target[1], Target[2]-y_shift);

		Log.d("Robot[state]:", "Start laser");
		api.laserControl(true);

		Log.d("Robot[state]:", "Start snapshot");
		api.takeSnapshot();
		api.laserControl(false);

		Log.d("Robot[state]:", "Start move");
		moveTo(10.46f, -8.65f, 4.65f, 0f, 0f, 0.707f, 0.707f);
		moveTo(10.6f, -8.0f, 4.5f, 0f, 0f, -0.707f, 0.707f);

		Log.d("Robot[state]:", "Stop mission");
		api.reportMissionCompletion();
	}
	@Override
	protected void runPlan2() {
	}
	@Override
	protected void runPlan3() {
	}
	public void moveTo(float px, float py, float pz, float qx, float qy, float qz, float qw)
	{
		Result result;
		int count = 0, max_count = 3;
		Point point = new Point(px, py, pz);
		Quaternion quaternion = new Quaternion(qx, qy, qz, qw);

		do
		{
			result = api.moveTo(point, quaternion, true);
			count++;
		}
		while (!result.hasSucceeded() && count < max_count);
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

			// String to double value convert
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
		finally
		{
			Log.d("StringParseToDouble[State]: ", "Finished");
		}
		// Double return : pattern, point x, point y, point z
		return new double[]{pattern, point_x, point_y, point_z};
	}
	public void moveTo(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des)
	{
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
	public double[] AR_event(int count_max)
	{
		int[] contents = new int[]{0, 0, 0, 0};
		int count = 0;
		boolean detected = false;
		double total_x = 0, total_y = 0;

		while (!detected && count < count_max)
		{
			Log.d("AR[status]:", " start");
			long start_time = SystemClock.elapsedRealtime();

			//////////////////////////////////////////////////////////////////////////////////////////////////////
			Mat source = api.getMatNavCam();
			Mat ids = new Mat();
			Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
			List<Mat> corners = new ArrayList<>();

			try
			{
				Aruco.detectMarkers(source, dictionary, corners, ids);
				contents[0] = (int) ids.get(0, 0)[0];
				contents[1] = (int) ids.get(1, 0)[0];
				contents[2] = (int) ids.get(2, 0)[0];
				contents[3] = (int) ids.get(3, 0)[0];

				if(contents[0] != 0 && contents[1] != 0 && contents[2] != 0 && contents[3] != 0)
				{
					detected = true;
					Log.d("AR[status]:", " Detected");
					Log.d("AR[Info]: ", contents[0] + ", " + contents[1] + ", " + contents[2] + ", " + contents[3]);

					double[][] Target_corners = new double[4][4];

					for(int i=0; i<4; i++)
					{
						int x1 = (int) corners.get(i).get(0, 0)[0];
						int y1 = (int) corners.get(i).get(0, 0)[1];
						int x2 = (int) corners.get(i).get(0, 2)[0];
						int y2 = (int) corners.get(i).get(0, 2)[1];
						int x3 = (int) corners.get(i).get(0, 1)[0];
						int y3 = (int) corners.get(i).get(0, 1)[1];
						int x4 = (int) corners.get(i).get(0, 3)[0];
						int y4 = (int) corners.get(i).get(0, 3)[1];

						Log.d("AR[Corners_X]["+i+"]: ", x1 + ", " + x2 + ", " + x3 + ", " + x4);
						Log.d("AR[Corners_Y]["+i+"]: ", y1 + ", " + y2 + ", " + y3 + ", " + y4);
						total_x += x1 + x2 + x3 + x4;
						total_y += y1 + y2 + y3 + y4;
					}
				}
			}
			catch (Exception e)
			{
				Log.d("AR[status]:", " Not detected");
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////
			finally
			{
				total_x /= 16;
				total_y /= 16;
				Log.d("Target[X,Y]: ", "" + total_x + ", " + total_y);
			}
			Log.d("AR[status]:", " stop");
			long stop_time = SystemClock.elapsedRealtime();

			Log.d("AR[count]:", " " + count);
			Log.d("AR[total_time]:", " " + (stop_time - start_time) / 1000);
			count++;
		}
		return new double[]{total_x, total_y};
	}
	public String QR_event_fast(float px, float py, float pz, float qx, float qy, float qz, float qw, int count_max)
	{
		String contents = null;
		int count = 0;

		while(contents == null && count < count_max)
		{
			Log.d("QR[status]:", " start");
			long start_time = SystemClock.elapsedRealtime();

			moveTo(px, py, pz, qx, qy, qz, qw);
			Log.d("QR[status]:", " get Image");

			Bitmap source = api.getBitmapNavCam();

			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			try
			{
				int[] pixel = new int[source.getWidth() * source.getHeight()];
				source.getPixels(pixel, 0, source.getWidth(), 0, 0, source.getWidth(), source.getHeight());
				Image barcode = new Image(source.getWidth(), source.getHeight(), "RGB4");
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				barcode.setData(pixel);
				ImageScanner reader = new ImageScanner();
				reader.setConfig(Symbol.NONE, Config.ENABLE, 0);
				reader.setConfig(Symbol.QRCODE, Config.ENABLE, 1);
				reader.scanImage(barcode.convert("Y800"));
				////////////////////////////////////////////////////////////////////////////////////////////////////////
				SymbolSet syms = reader.getResults();
				for (Symbol sym : syms)
				{
					contents = sym.getData();
					Log.d("QR[status]:", " Detected");
					Log.d("QR[contents]:", " " + contents);
				}
			}
			catch (Exception error)
			{
				Log.e("QR[status]:", " Not detected");
			}
			////////////////////////////////////////////////////////////////////////////////////////////////////////////
			Log.d("QR[status]:", " stop");
			long stop_time = SystemClock.elapsedRealtime();

			Log.d("QR[count]:", " " + count);
			Log.d("QR[total_time]:", " " + (stop_time - start_time) / 1000);
			count++;
		}
		api.sendDiscoveredQR(contents);
		return contents;
	}
}
