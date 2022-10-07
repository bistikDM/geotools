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
public class ECEF extends Frame {
	/**
	 * A value in m.
	 */
	private double x;

	/**
	 * A value in m.
	 */
	private double y;

	/**
	 * A value in m.
	 */
	private double z;
}
