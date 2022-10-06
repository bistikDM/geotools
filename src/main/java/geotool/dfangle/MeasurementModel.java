package geotool.dfangle;

import java.util.function.Function;

import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;

/**
 * @author Bistik <br>
 *         <br>
 *         <b>Reference:</b> <br>
 *         <a href=
 *         "https://www.jhuapl.edu/content/techdigest/pdf/V31-N03/31-03-Grabbe.pdf">Geo-Location
 *         Using Direction Finding Angles</a>
 */
public final class MeasurementModel {
	private static final double EARTH_ECCENTRICITY_SQUARED = 0.01671d * 0.01671d;
	private static final double EARTH_RADIUS_METER = 6.371e6d;

	private static Function<Double, Double> transverseRadius = (latitude) -> {
		double sinLat = FastMath.sin(latitude);
		double denom = FastMath.sqrt(1.0d - MeasurementModel.EARTH_ECCENTRICITY_SQUARED * sinLat * sinLat);

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
}
