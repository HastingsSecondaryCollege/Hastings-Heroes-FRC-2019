package frc.utils;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public final class HelperFunctions {
	/** Prevent this class from being instantiated. */
	private HelperFunctions() {
	}

	/**
	 * Limits the given input to the given magnitude.
	 */
	public static double limit(double v, double maxMagnitude) {
		return limit(v, -maxMagnitude, maxMagnitude);
	}

	public static double limit(double v, double min, double max) {
		return Math.min(max, Math.max(min, v));
	}

	public static String joinStrings(String delim, List<?> strings) {
		StringBuilder sb = new StringBuilder();
		for (int i = 0; i < strings.size(); ++i) {
			sb.append(strings.get(i).toString());
			if (i < strings.size() - 1) {
				sb.append(delim);
			}
		}
		return sb.toString();
	}

	public static boolean epsilonEquals(double a, double b, double epsilon) {
		return (a - epsilon <= b) && (a + epsilon >= b);
	}

	public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
		boolean result = true;
		for (Double value_in : list) {
			result &= epsilonEquals(value_in, value, epsilon);
		}
		return result;
	}

	public static double standardDeviation(double[] arr) {
		double mean = 0.0;
		double[] temp = new double[arr.length];

		mean = mean(arr);

		for (int i = 0; i < temp.length; i++) {
			temp[i] = Math.pow((arr[i] - mean), 2);
		}

		return Math.sqrt(mean(temp));
	}

	public static double mean(double[] arr) {
		double sum = 0.0;

		for (int i = 0; i < arr.length; i++) {
			sum += arr[i];
		}

		return sum / arr.length;
	}

	public static double max(double[] arr) {
		double max = 0;
		for (int i = 0; i < arr.length; i++) {
			if (arr[i] > max) {
				max = arr[i];
			}
		}

		return max;
	}
}