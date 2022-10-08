package geotool.dfangle;

import java.util.function.BiFunction;
import java.util.function.Function;

import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;

import geotool.pojo.AntennaArray;
import geotool.pojo.Body;
import geotool.pojo.ECEF;
import geotool.pojo.LLA;
import lombok.NonNull;

public class MeasurementMatrix {

	private static BiFunction<Double, Double, RealMatrix> azimuth = (elevation, azimuth) -> {
		RealMatrix matrix = MatrixUtils
				.createRowRealMatrix(new double[] { -FastMath.sin(azimuth), FastMath.cos(azimuth), 0.0d });

		return matrix.scalarMultiply(1.0d / FastMath.cos(elevation));
	};

	private static BiFunction<Double, Double, RealMatrix> elevation = (elevation, azimuth) -> {
		return MatrixUtils.createRowRealMatrix(new double[] { -FastMath.cos(azimuth) * FastMath.sin(elevation),
				-FastMath.cos(azimuth) * FastMath.sin(elevation), -FastMath.cos(elevation) });
	};

	private static TriFunction<Double, Double, Double, RealMatrix> aoa = (elevation, azimuth, aoa) -> {
		RealMatrix matrix = MatrixUtils.createRowRealMatrix(new double[] { -FastMath.pow(FastMath.sin(aoa), 2.0d),
				FastMath.sin(azimuth) * FastMath.cos(elevation) * FastMath.cos(aoa),
				-FastMath.sin(elevation) * FastMath.cos(aoa) });

		return matrix.scalarMultiply(1.0d / FastMath.sin(aoa));
	};

	private static BiFunction<Double, AntennaArray, RealMatrix> secondFactor = (relativeMagnitude, los) -> {
		RealMatrix identity = MatrixUtils.createRealIdentityMatrix(3);
		RealMatrix losMatrix = MatrixUtils
				.createColumnRealMatrix(new double[] { los.getAlpha(), los.getBeta(), los.getGamma() });

		return identity.subtract(losMatrix)
				.multiplyTransposed(losMatrix)
				.scalarMultiply(1.0d / relativeMagnitude);
	};

	/**
	 * Not used while altitude is considered constant.
	 */
	private static Function<Double, Double> derivedTransverse = (latitude) -> {
		double transverse = MeasurementModel.transverseRadius.apply(latitude);
		double nom = MeasurementModel.EARTH_ECCENTRICITY_SQUARED * FastMath.sin(latitude) * FastMath.cos(latitude)
				* transverse;
		double den = 1.0d - MeasurementModel.EARTH_ECCENTRICITY_SQUARED * FastMath.pow(FastMath.sin(latitude), 2.0d);

		return nom / den;
	};

	/**
	 * Altitude is considered constant.
	 */
	private static TriFunction<Double, Double, Double, RealMatrix> fifthFactor = (latitude, longitude, altitude) -> {
		double transverse = MeasurementModel.transverseRadius.apply(latitude);

		double tgtLonFirst = -(transverse * altitude) * FastMath.sin(longitude) * FastMath.cos(latitude);
		double tgtLonSecond = (transverse + altitude) * FastMath.cos(longitude) * FastMath.cos(latitude);
		RealMatrix tgtLon = MatrixUtils.createColumnRealMatrix(new double[] { tgtLonFirst, tgtLonSecond, 0.0d });

		double tgtLatFirst = -(transverse + altitude) * FastMath.cos(longitude) * FastMath.sin(latitude);
		double tgtLatSecond = -(transverse + altitude) * FastMath.sin(longitude) * FastMath.sin(latitude);
		double tgtLatThird = (transverse * (1 - MeasurementModel.EARTH_ECCENTRICITY_SQUARED) + altitude)
				* FastMath.cos(latitude);
		RealMatrix tgtLat = MatrixUtils.createColumnRealMatrix(new double[] { tgtLatFirst, tgtLatSecond, tgtLatThird });

		return tgtLon.multiply(tgtLat);
	};

	private MeasurementMatrix() {

	}

	/**
	 * Calculates the 3 X 3 measurement matrix for the azimuth measurement.
	 * 
	 * @param origin
	 * @param originPos
	 * @param originBody
	 * @param originAntenna
	 * @param target
	 * @return A 2D {@code double[3][3]} array for the azimuth measurement.
	 */
	public static double[][] getAzimuthMeasurement(@NonNull ECEF origin, @NonNull LLA originPos,
			@NonNull Body originBody, @NonNull AntennaArray originAntenna, @NonNull LLA target) {
		ECEF relativePos = MeasurementModel.getRelativePosition(origin, target);
		AntennaArray los = MeasurementModel.getLOS(originPos, originBody, originAntenna, relativePos);
		double[] values = MeasurementMatrix.getValues(los);
		RealMatrix first = MeasurementMatrix.azimuth.apply(values[0], values[1]);

		return MeasurementMatrix.getMeasurementMatrix(first, originPos, originBody, originAntenna, relativePos, los,
				target);
	}

	/**
	 * Calculates the 3 X 3 measurement matrix for the elevation measurement.
	 * 
	 * @param origin
	 * @param originPos
	 * @param originBody
	 * @param originAntenna
	 * @param target
	 * @return A 2D {@code double[3][3]} array for the elevation measurement.
	 */
	public static double[][] getElevationMeasurement(@NonNull ECEF origin, @NonNull LLA originPos,
			@NonNull Body originBody, @NonNull AntennaArray originAntenna, @NonNull LLA target) {
		ECEF relativePos = MeasurementModel.getRelativePosition(origin, target);
		AntennaArray los = MeasurementModel.getLOS(originPos, originBody, originAntenna, relativePos);
		double[] values = MeasurementMatrix.getValues(los);
		RealMatrix first = MeasurementMatrix.elevation.apply(values[0], values[1]);

		return MeasurementMatrix.getMeasurementMatrix(first, originPos, originBody, originAntenna, relativePos, los,
				target);
	}

	/**
	 * Calculates the 3 X 3 measurement matrix for the AOA measurement.
	 * 
	 * @param origin
	 * @param originPos
	 * @param originBody
	 * @param originAntenna
	 * @param target
	 * @return A 2D {@code double[3][3]} array for the AOA measurement.
	 */
	public static double[][] getAOAMeasurement(@NonNull ECEF origin, @NonNull LLA originPos, @NonNull Body originBody,
			@NonNull AntennaArray originAntenna, @NonNull LLA target) {
		ECEF relativePos = MeasurementModel.getRelativePosition(origin, target);
		AntennaArray los = MeasurementModel.getLOS(originPos, originBody, originAntenna, relativePos);
		double[] values = MeasurementMatrix.getValues(los);
		RealMatrix first = MeasurementMatrix.aoa.apply(values[0], values[1], values[2]);

		return MeasurementMatrix.getMeasurementMatrix(first, originPos, originBody, originAntenna, relativePos, los,
				target);
	}

	private static double[] getValues(AntennaArray los) {
		double elevation = MeasurementModel.getElevation(los);
		double azimuth = MeasurementModel.getAzimuth(los);
		double aoa = MeasurementModel.getAOA(los);

		return new double[] { elevation, azimuth, aoa };
	}

	private static RealMatrix getAntennaVector(LLA originPos, Body originBody, AntennaArray originAntenna) {
		double[][] ba = RotationMatrix.TBA(originAntenna.getAlpha(), originAntenna.getBeta(), originAntenna.getGamma());
		double[][] en = RotationMatrix.TEN(originPos.getLongitude(), originPos.getLatitude());
		double[][] nb = RotationMatrix.TNB(originBody.getYaw(), originBody.getPitch(), originBody.getRoll());
		double[][] eb = RotationMatrix.TEB(en, nb);
		double[][] ea = RotationMatrix.TEA(eb, ba);

		return MatrixUtils.createRealMatrix(RotationMatrix.TAE(ea));
	}

	private static double[][] getMeasurementMatrix(RealMatrix first, LLA originPos, Body originBody,
			AntennaArray originAntenna, ECEF relativePos, AntennaArray los, LLA target) {
		RealMatrix relative = MatrixUtils
				.createColumnRealMatrix(new double[] { relativePos.getX(), relativePos.getY(), relativePos.getZ() });
		RealMatrix ae = MeasurementMatrix.getAntennaVector(originPos, originBody, originAntenna);
		RealMatrix aFrame = ae.multiply(relative);
		double[] aRelative = aFrame.getColumn(1);
		double magnitude = FastMath.sqrt(
				FastMath.pow(aRelative[0], 2.0d) + FastMath.pow(aRelative[1], 2.0d) + FastMath.pow(aRelative[2], 2.0d));
		RealMatrix second = MeasurementMatrix.secondFactor.apply(magnitude, los);
		RealMatrix fourth = MatrixUtils.createRealIdentityMatrix(3);
		RealMatrix fifth = MeasurementMatrix.fifthFactor.apply(target.getLatitude(), target.getLongitude(),
				target.getAltitude());

		return first.multiply(second)
				.multiply(ae)
				.multiply(fourth)
				.multiply(fifth)
				.getData();
	}
}
