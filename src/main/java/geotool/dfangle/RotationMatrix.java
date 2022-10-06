package geotool.dfangle;

import java.util.function.BiFunction;
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
public final class RotationMatrix {

	private static Function<Double, RealMatrix> rx = (val) -> {
		double[][] rx = { { 1.0d, 0.0d, 0.0d }, { 0.0d, FastMath.cos(val), -FastMath.sin(val) },
				{ 0.0d, FastMath.sin(val), FastMath.cos(val) } };

		return MatrixUtils.createRealMatrix(rx);
	};

	private static Function<Double, RealMatrix> ry = (val) -> {
		double[][] ry = { { FastMath.cos(val), 0.0d, FastMath.sin(val) }, { 0.0d, 1.0d, 0.0d },
				{ -FastMath.sin(val), 0.0d, FastMath.cos(val) } };

		return MatrixUtils.createRealMatrix(ry);
	};

	private static Function<Double, RealMatrix> rz = (val) -> {
		double[][] rz = { { FastMath.cos(val), -FastMath.sin(val), 0.0d },
				{ FastMath.sin(val), FastMath.cos(val), 0.0d }, { 0.0d, 0.0d, 1.0d } };

		return MatrixUtils.createRealMatrix(rz);
	};

	private static BiFunction<double[][], double[][], RealMatrix> genericRotation = (first, second) -> {
		if (!RotationMatrix.isValid(first, second)) {
			throw new IllegalArgumentException("Array is either null or incorrect size.");
		}

		RealMatrix firstMatrix = MatrixUtils.createRealMatrix(first);
		RealMatrix secondMatrix = MatrixUtils.createRealMatrix(second);

		return firstMatrix.multiply(secondMatrix);
	};

	private static Function<double[][], RealMatrix> genericTranspose = (arr) -> {
		if (!RotationMatrix.isNotEmptyOrNull(arr)) {
			throw new IllegalArgumentException("Array is either null or incorrect size.");
		}

		RealMatrix matrix = MatrixUtils.createRealMatrix(arr);

		return matrix.transpose();
	};

	private RotationMatrix() {

	}

	/**
	 * Transformation to ECEF frame from NED frame.
	 * 
	 * @param longitude The longitude in radians.
	 * @param latitude  The geodetic latitude in radians.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TEN(double longitude, double latitude) {
		double lat = -latitude - (FastMath.PI / 2);
		RealMatrix rz = RotationMatrix.rz.apply(longitude);
		RealMatrix ry = RotationMatrix.ry.apply(lat);

		return rz.multiply(ry).getData();
	}

	/**
	 * Transformation to NED frame from aircraft body frame.
	 * 
	 * @param yaw   The yaw in radians.
	 * @param pitch The pitch in radians.
	 * @param roll  The roll in radians.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TNB(double yaw, double pitch, double roll) {
		RealMatrix rx = RotationMatrix.rx.apply(roll);
		RealMatrix ry = RotationMatrix.ry.apply(pitch);
		RealMatrix rz = RotationMatrix.rz.apply(yaw);

		return rz.multiply(ry).multiply(rx).getData();
	}

	/**
	 * Transformation to aircraft body frame from antenna array frame.
	 * 
	 * @param alpha The alpha in radians.
	 * @param beta  The beta in radians.
	 * @param gamma The gamma in radians.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TBA(double alpha, double beta, double gamma) {
		RealMatrix rx = RotationMatrix.rx.apply(gamma);
		RealMatrix ry = RotationMatrix.ry.apply(beta);
		RealMatrix rz = RotationMatrix.rz.apply(alpha);

		return rz.multiply(ry).multiply(rx).getData();
	}

	/**
	 * Transformation to ECEF frame from aircraft body frame.
	 * 
	 * @param enFrame A 3 by 3 array of the ECEF frame typically obtained from
	 *                {@link RotationMatrix#TEN(double, double)}.
	 * @param nbFrame A 3 by 3 array of the aircraft body frame typically obtained
	 *                from {@link RotationMatrix#TNB(double, double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TEB(double[][] enFrame, double[][] nbFrame) {
		return RotationMatrix.genericRotation.apply(enFrame, nbFrame).getData();
	}

	/**
	 * Transformation to ECEF frame from antenna array frame.
	 * 
	 * @param ebFrame A 3 by 3 array of the ECEF frame typically obtained from
	 *                {@link RotationMatrix#TEB(double[][], double[][])}.
	 * @param baFrame A 3 by 3 array of the aircraft body frame typically obtained
	 *                from {@link RotationMatrix#TBA(double, double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TEA(double[][] ebFrame, double[][] baFrame) {
		return RotationMatrix.genericRotation.apply(ebFrame, baFrame).getData();
	}

	/**
	 * Transformation to NED frame from ECEF frame.
	 * 
	 * @param enFrame A 3 by 3 array of the ECEF frame typically obtained from
	 *                {@link RotationMatrix#TEN(double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TNE(double[][] enFrame) {
		return RotationMatrix.genericTranspose.apply(enFrame).getData();
	}

	/**
	 * Transformation to NED frame from antenna array frame.
	 * 
	 * @param nbFrame A 3 by 3 array of the NED frame typically obtained from
	 *                {@link RotationMatrix#TNB(double, double, double)}.
	 * @param baFrame A 3 by 3 array of the aircraft body frame typically obtained
	 *                from {@link RotationMatrix#TBA(double, double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TNA(double[][] nbFrame, double[][] baFrame) {
		return RotationMatrix.genericRotation.apply(nbFrame, baFrame).getData();
	}

	/**
	 * Transformation to aircraft body frame from ECEF frame.
	 * 
	 * @param ebFrame A 3 by 3 array of the ECEF frame typically obtained from
	 *                {@link RotationMatrix#TEB(double[][], double[][])}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TBE(double[][] ebFrame) {
		return RotationMatrix.genericTranspose.apply(ebFrame).getData();
	}

	/**
	 * Transformation to aircraft body frame from NED frame.
	 * 
	 * @param nbFrame A 3 by 3 array of the NED frame typically obtained from
	 *                {@link RotationMatrix#TNB(double, double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TBN(double[][] nbFrame) {
		return RotationMatrix.genericTranspose.apply(nbFrame).getData();
	}

	/**
	 * Transformation to antenna array frame from ECEF frame.
	 * 
	 * @param eaFrame A 3 by 3 array of the ECEF frame typically obtained from
	 *                {@link RotationMatrix#TEA(double[][], double[][])}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TAE(double[][] eaFrame) {
		return RotationMatrix.genericTranspose.apply(eaFrame).getData();
	}

	/**
	 * Transformation to antenna array frame from NED frame.
	 * 
	 * @param naFrame A 3 by 3 array of the NED frame typically obtained from
	 *                ({@link RotationMatrix#TNA(double[][], double[][])}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TAN(double[][] naFrame) {
		return RotationMatrix.genericTranspose.apply(naFrame).getData();
	}

	/**
	 * Transformation to antenna array frame from aircraft body frame.
	 * 
	 * @param baFrame A 3 by 3 array of the aircraft body frame typically obtained
	 *                from {@link RotationMatrix#TBA(double, double, double)}.
	 * @return A 3 by 3 array of the transformed frame.
	 */
	public static double[][] TAB(double[][] baFrame) {
		return RotationMatrix.genericTranspose.apply(baFrame).getData();
	}

	private static boolean isNotEmptyOrNull(double[][] arr) {
		if (arr == null || arr.length != 3) {
			return false;
		}

		for (double[] sub : arr) {
			if (sub.length != 3) {
				return false;
			}
		}

		return true;
	}

	private static boolean isValid(double[][] first, double[][] second) {
		return (RotationMatrix.isNotEmptyOrNull(first) && RotationMatrix.isNotEmptyOrNull(second));
	}
}
