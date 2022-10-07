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
public class Body extends Frame {
	/**
	 * A value in radian.
	 */
	private double yaw;

	/**
	 * A value in radian.
	 */
	private double pitch;

	/**
	 * A value in radian.
	 */
	private double roll;
}
