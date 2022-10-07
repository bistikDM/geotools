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
public class NED extends Frame {
	/**
	 * A value in m.
	 */
	private double north;

	/**
	 * A value in m.
	 */
	private double east;

	/**
	 * A value in m.
	 */
	private double down;
}
