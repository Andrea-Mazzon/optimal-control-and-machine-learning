package it.univr.controlledmarkovchains.handout1solution;

import java.util.stream.IntStream;

/**
 * The main contribution of this class is to provide the solution of the gambler problem when the
 * probability of getting head is known. It does it by extending the class PolicyIteration,
 * providing the implementation of its abstract methods.
 * 
 * @author Andrea Mazzon
 *
 */
public class CoinGamblerProblemWithPolicyIteration extends PolicyIteration {

	
	private double headProbability;
	
	private int amountOfMoneyToReach;
	
	
	/**
	 * It constructs an object to compute the solution of the gambler problem with known head probability.
	 * 
	 * @param discountFactor: the discount factor gamma in the notes
	 * @param requiredPrecision: the iterations stop when the absolute value of the difference between the new and past
	 * 		  values of the value function is smaller than requiredPrecision for all the entries
	 * @param headProbability, the probability to get head
	 * @param amountOfMoneyToReach, the amount that the capital process must hit in order for the gambler to win the bet
	 */
	public CoinGamblerProblemWithPolicyIteration(double discountFactor, double requiredPrecision, double headProbability, int amountOfMoneyToReach) {
		super(IntStream.range(0, amountOfMoneyToReach+1).asDoubleStream().toArray(), //the vector (0,1,2,...,amountOfMoneyToReach)
				new double[] {0.0, 1.0}, //the rewards at absorbing states
				new int[] {0, amountOfMoneyToReach}, //the absorbing states
				discountFactor, 
				requiredPrecision);
		this.headProbability = headProbability;
		this.amountOfMoneyToReach = amountOfMoneyToReach;
	}

	
	@Override
	protected double[] computeActions(double state) {
		/*
		 * Possible actions are (1,2,..,n) where n is the minimum between the capital (we cannot go negative) and the capital
		 * needed to reach amountOfMoneyToReach (it does not make sense to invest more). We write +1 because the second number in range
		 * is exclusive
		 */
        double[] actions = IntStream.range(1, (int) (Math.min(state, amountOfMoneyToReach - state) + 1)).asDoubleStream().toArray();
        
		return actions;
	}

	@Override
	protected double[] computeExpectedReturnsForStateAndActions(double state, double[] actions) {
				
		double[] oldStateValues = getOldValueFunctions();
		double discountFactor = getDiscountFactor();
		double[] actionReturns = new double[actions.length];
        for (int actionIndex = 0; actionIndex < actions.length; actionIndex ++ ) {
        	//the expected value at the next step given the chosen action and the current state. There is no reward function
        	actionReturns[actionIndex]= discountFactor*(headProbability * 
        			oldStateValues[ (int) (state + actions[actionIndex])]
        				 + (1 - headProbability) * oldStateValues[(int) (state - actions[actionIndex])]);      	
        }
		return actionReturns;
	}

	/*
	 * This is the identity matrix minus the matrix representing the transition probabilities between
	 * the non-absorsing states
	 */
	@Override
	protected double[][] computeSystemMatrix(double[] actions) {
		
		//amountOfMoneyToReach -1 is the number of non-absorbing states
		double[][] systemMatrix = new double[amountOfMoneyToReach -1][amountOfMoneyToReach -1];
		
		//in our problem, indices of non-absorbing states go from 1 to amountOfMoneyToReach -1
		for (int startingStateIndex = 1; startingStateIndex<=amountOfMoneyToReach -1; startingStateIndex++) {
				//if actions[startingStateIndex]==startingStateIndex, it contributes to the constant term
				if (startingStateIndex - actions[startingStateIndex]>0) {	
					//the index of the row where we store it is startingStateIndex-1
					systemMatrix[startingStateIndex-1][startingStateIndex - 1 - (int) actions[startingStateIndex]]=
							-(1-headProbability);
				}
				if (startingStateIndex + actions[startingStateIndex]<amountOfMoneyToReach) {
					//the index of the row where we store it is startingStateIndex-1
					systemMatrix[startingStateIndex-1][startingStateIndex - 1 + (int) actions[startingStateIndex]]=
							-headProbability;
				}	
				
				//contribute of the identity matrix
				systemMatrix[startingStateIndex-1][startingStateIndex - 1]=1.0;
		}
		return systemMatrix;
	}

	@Override
	protected double[] computeConstantTerm(double[] actions) {
		double[] constantTerm = new double[amountOfMoneyToReach -1];
		/*
		 * The running rewards are zero: the only non-zero terms are possibly given by transition
		 * probabilities towards absorbing states
		 */
		for (int stateIndex = 1; stateIndex<=amountOfMoneyToReach -1; stateIndex++) {
			/*
			 * More in particular, since the final reward in 0 is 0, actually the only
			 * non-zero terms are given by the transition probability towards the last state
			 */
			if (actions[stateIndex]==amountOfMoneyToReach - stateIndex) {
				constantTerm[stateIndex-1]=headProbability;
			}
		}
		return constantTerm;
	}
}	