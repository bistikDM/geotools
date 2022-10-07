package geotool.dfangle;

import java.util.function.Function;

import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;

import geotool.pojo.AntennaArray;
import geotool.pojo.Body;
import geotool.pojo.ECEF;
import geotool.pojo.LLA;
import lombok.NonNull;

/**
 * @author Bistik <br>
 *         <br>
 *         <b>Reference:</b> <br>
 *         <a href=
 *         "https://www.jhuapl.edu/content/techdigest/pdf/V31-N03/31-03-Grabbe.pdf">Geo-Location
 *         Using Direction Finding Angles</a>
 */
public final class MeasurementModel {
	private static final double EARTH_ECCENTRICITY_SQUARED = FastMath.pow(0.01671d, 2.0d);
	private static final double EARTH_RADIUS_METER = 6.371e6d;

	private static Function<Double, Double> transverseRadius = (latitude) -> {
		double sinLat = FastMath.sin(latitude);
		double denom = FastMath.sqrt(1.0d - MeasurementModel.EARTH_ECCENTRICITY_SQUARED * FastMath.pow(sinLat, 2.0d));

		return MeasurementModel.EARTH_RADIUS_METER / denom;
	};

	private static TriFunction<Double, Double, Double, RealMatrix> ecefLocation = (longitude, latitude, altitude) -> {
		double gamma = MeasurementModel.transverseRadius.apply(latitude);
		double x = (gamma + altitude) * FastMath.cos(longitude) * FastMath.cos(latitude);
		double y = (gamma + altitude) * FastMath.sin(longitude) * FastMath.cos(latitude);
		double z = (gamma * (1.0d - MeasurementModel.EARTH_ECCENTRICITY_SQUARED) + altitude) * FastMath.sin(latitude);
		double[] ecef = { x, y, z };

		return MatrixUtils.createColumnRealMatrix(ecef);
	};

	private MeasurementModel() {

	}

	/**
	 * Calculates the position of the target relative to the origin in ECEF
	 * coordinates.
	 * 
	 * @param origin The origin's position.
	 * @param target The target's position.
	 * @return The position of the target relative to the origin in ECEF.
	 */
	public static ECEF getRelativePosition(@NonNull ECEF origin, @NonNull LLA target) {
		RealMatrix originMatrix = MatrixUtils
				.createColumnRealMatrix(new double[] { origin.getX(), origin.getY(), origin.getZ() });
		RealMatrix targetMatrix = MeasurementModel.ecefLocation.apply(target.getLongitude(), target.getLatitude(),
				target.getAltitude());
		double[] result = targetMatrix.subtract(originMatrix)
				.getColumn(1);

		return ECEF.builder()
				.x(result[0])
				.y(result[1])
				.z(result[2])
				.build();
	}

	/**
	 * Calculates the unit vector along the LOS based on the antenna array.
	 * 
	 * @param originPos     The origin's positon.
	 * @param originBody    The origin's body orientation.
	 * @param originAntenna The origin's antenna position.
	 * @param relativePos   The relative position of the target to the origin.
	 * @return A unit vector along the LOS in frame A.
	 */
	public static AntennaArray getLOS(@NonNull LLA originPos, @NonNull Body originBody,
			@NonNull AntennaArray originAntenna, @NonNull ECEF relativePos) {
		RealMatrix relative = MatrixUtils
				.createColumnRealMatrix(new double[] { relativePos.getX(), relativePos.getY(), relativePos.getZ() });

		// Intermediate calculations.
		double[][] ba = RotationMatrix.TBA(originAntenna.getAlpha(), originAntenna.getBeta(), originAntenna.getGamma());
		double[][] en = RotationMatrix.TEN(originPos.getLongitude(), originPos.getLatitude());
		double[][] nb = RotationMatrix.TNB(originBody.getYaw(), originBody.getPitch(), originBody.getRoll());
		double[][] eb = RotationMatrix.TEB(en, nb);
		double[][] ea = RotationMatrix.TEA(eb, ba);

		// calculate relative position vector in antenna array frame.
		RealMatrix ae = MatrixUtils.createRealMatrix(RotationMatrix.TAE(ea));
		RealMatrix aFrame = ae.multiply(relative);
		double[] aRelative = aFrame.getColumn(1);
		double magnitude = FastMath.sqrt(
				FastMath.pow(aRelative[0], 2.0d) + FastMath.pow(aRelative[1], 2.0d) + FastMath.pow(aRelative[2], 2.0d));
		double[] result = aFrame.scalarMultiply(1.0d * magnitude)
				.getColumn(1);

		return AntennaArray.builder()
				.alpha(result[0])
				.beta(result[1])
				.gamma(result[2])
				.build();
	}

	/**
	 * Calculates the DF angle azimuth in radian.
	 * 
	 * @param array The antenna array to calculate the DF angle.
	 * @return A radian value such that if the antenna frame used to calculate the
	 *         {@code array} is aligned with the body frame, a positive azimuth
	 *         indicates that the target is to the right of origin.
	 */
	public static double getAzimuth(@NonNull AntennaArray array) {
		return FastMath.atan((array.getBeta() / array.getAlpha()));
	}

	/**
	 * Calculates the elevation in m.
	 * 
	 * @param array The antenna array to calculate the elevation.
	 * @return A meter value such that if the antenna frame used to calculate the
	 *         {@code array} is aligned with the body frame, a positive elevation
	 *         indicates that the target is above the origin.
	 */
	public static double getElevation(@NonNull AntennaArray array) {
		return FastMath.atan(-array.getGamma()
				/ (FastMath.sqrt(FastMath.pow(array.getAlpha(), 2.0d) + FastMath.pow(array.getBeta(), 2.0d))));
	}

	public static double getAOA(AntennaArray array) {
		return FastMath.atan(FastMath.sqrt(FastMath.pow(array.getBeta(), 2.0d) + FastMath.pow(array.getGamma(), 2.0d))
				/ array.getAlpha());
	}
}
