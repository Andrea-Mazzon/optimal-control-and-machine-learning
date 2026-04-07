package it.univr.controlledmarkovchains.handout2solution;



import java.util.Arrays;
import java.util.function.DoubleUnaryOperator;

import net.finmath.plots.Named;
import net.finmath.plots.Plot2D;

public class CoinGamblerProblemWithTemporalDifferenceLearningTest {

	/**
	 * This class tests the implementation of TemporalDifferenceLearning and its
	 * derived classes CoinGamblerProblemWithQLearningInheritance and
	 * CoinGamblerProblemWithSarsaInheritance
	 * 
	 * @author Andrea Mazzon
	 *
	 */
	public static void main(String[] args) {

		
		double headProbability = 0.4;
		int moneyToWin = 100;
		double discountFactor = 1.0;//no discount

		int numberOfEpisodes = 10000;	
		double learningRate = 0.3;
		double explorationProbability = 0.3;//usually, one needs to choose a small exploration probability

		TemporalDifferenceLearning problemSolver = new CoinGamblerProblemWithSarsaInheritance(headProbability, discountFactor, moneyToWin, numberOfEpisodes, learningRate, explorationProbability);
		//TemporalDifferenceLearning problemSolver = new CoinGamblerProblemWithQLearningInheritance(headProbability, discountFactor, moneyToWin, numberOfEpisodes, learningRate, explorationProbability);

		
		double[] valueFunctions = problemSolver.getValueFunctions();
		
		
		final Plot2D plotValueFunctions = new Plot2D(1, moneyToWin-1, moneyToWin-1, Arrays.asList(
				new Named<DoubleUnaryOperator>("Value function", x -> valueFunctions[(int) x])));
		
		plotValueFunctions.setXAxisLabel("State");
		plotValueFunctions.setYAxisLabel("Value function");

		plotValueFunctions.show();
		
		int[] optimalActionsIndices = problemSolver.getOptimalActionsIndices();
		
		
		//the action is equal to the index of the action plus 1 (because we start from 0)
		final Plot2D plotOptimalActions = new Plot2D(1, moneyToWin-1, moneyToWin-1, Arrays.asList(
				new Named<DoubleUnaryOperator>("Optimal action", x -> optimalActionsIndices[(int) x]+1)));
		
		plotOptimalActions.setXAxisLabel("State");
		plotOptimalActions.setYAxisLabel("Optimal money investment on head");

		plotOptimalActions.show();

	}
}

