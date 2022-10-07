package geotool.pojo;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.ToString;

@NoArgsConstructor
@AllArgsConstructor
@Getter
@Builder
@ToString
public class AntennaArray extends Frame {
	/**
	 * A value in radian (mounting angle).
	 */
	private double alpha;

	/**
	 * A value in radian (mounting angle).
	 */
	private double beta;

	/**
	 * A value in radian (mounting angle).
	 */
	private double gamma;
}
