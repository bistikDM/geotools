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
public class LLA {

	/**
	 * Geodetic longitude in radian.
	 */
	private double longitude;

	/**
	 * Geodetic latitude in radian.
	 */
	private double latitude;

	/**
	 * Geodetic altitude in m.
	 */
	private double altitude;
}
