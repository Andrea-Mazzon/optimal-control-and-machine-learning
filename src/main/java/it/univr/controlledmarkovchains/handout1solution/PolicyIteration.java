package it.univr.controlledmarkovchains.handout1solution;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import it.univr.usefulmethodsarrays.UsefulMethodsArrays;

/**
 * Main goal of this class is to provide the solution of a stochastic control problem in the setting
 * of controlled Markov chains for discrete time and discrete space, under the hypothesis that the transition
 * probabilities from one state to the other (i.e., the probabilities defining "the environment") are known.
 * The representation of the problem with the Bellman equation is used in order to iteratively get the
 * optimal actions and the associated value functions, that is, the values achieved with the "best action"
 * in every state. Once the new update of the optimal actions are computed, the associated value function
 * values are obtained by solving a linear system.
 *  
 * @author Andrea Mazzon
 *
 */
public abstract class PolicyIteration {

	//the possible states of the system: given in the constructor
	private double[] states;

	//indices of the absorbing states: given in the constructor
	private int[] absorbingStatesIndices;
	
	//computed once the indices of the absorbing states are given
	private int[] nonAbsorbingStatesIndices;
	
	//the final rewards for every absorbing state: given in the constructor
	private double[] rewardsAtAbsorbingStates;

	//the discount factor gamma in the notes: given in the constructor
	private double discountFactor;

	/*
	 * The iterations stop when the absolute value of the difference between the new and past values
	 * of the value function is smaller than requiredPrecision for all the entries.
	 * Given in the constructor
	 */ 
	private double requiredPrecision;


	/*
	 * This array will contain the value function for every state, that is, the maximized values.
	 * It will get updated until the difference with past values is small enough.
	 */
	private double[] valueFunctions;

	/*
	 * This array will contain the optimal actions for every state. It will get updated until the
	 * difference between new and past values of the value functions is small enough.
	 */
	private double[] optimalActions;

	private int numberOfStates;

	//it will be udpated during the loop, and contains the past values for every state
	private double[] oldValueFunctions;

	//a list of double arrays containing the value functions which are computed during the loop
	private ArrayList<double[]> updatedValueFunctions = new ArrayList<double[]>();


	/**
	 * It constructs an object to solve a stochastic control problem in the setting of controlled Markov chains
	 * for discrete time and discrete space, under the hypothesis that the transition probabilities from one state
	 * to the other (i.e., the probabilities defining "the environment") are known.
	 * 
	 * @param states, the possible states of the system
	 * @param rewardsAtAbsorbingStates, the rewards for every absorbing state.
	 * @param absorbingStatesIndices, the indices of states which are absorbing: for example, for the gambler problem
	 * 		  they are 0 and the last index.
	 * @param discountFactor, the discount factor gamma in the notes
	 * @param requiredPrecision, the iterations stop when the absolute value of the difference between the new and past
	 * 		  values of the value function is smaller than requiredPrecision for all the entries
	 */
	public PolicyIteration(double[] states, double[] rewardsAtAbsorbingStates, int[] absorbingStatesIndices, double discountFactor, double requiredPrecision) {

		this.states = states;
		numberOfStates = states.length;

		this.rewardsAtAbsorbingStates = rewardsAtAbsorbingStates;
		this.absorbingStatesIndices = absorbingStatesIndices;

		//to be used only for computing more easily nonAbsorbingStatesIndices, not in other methods
		List<Integer> absorbingStatesIndicesAsList = Arrays.stream(absorbingStatesIndices).boxed().toList();

		nonAbsorbingStatesIndices = IntStream.range(0, numberOfStates)
				.filter(i -> !absorbingStatesIndicesAsList.contains(i))
				.toArray();

		this.discountFactor = discountFactor; 

		this.requiredPrecision = requiredPrecision; 
	}

	/*
	 * This is a private method which is used to iteratively compute the optimizing actions
	 * and the associated value functions for every state.
	 */
	private void generateValueFunctionsAndOptimalActions() throws Exception {
		
		valueFunctions = new double[numberOfStates];

		//first we assign the value functions at absorbing states. Not to be changed afterwards 
		for (int i = 0; i < absorbingStatesIndices.length; i++) {
			valueFunctions[absorbingStatesIndices[i]] = rewardsAtAbsorbingStates[i];
		}

		//first "guess" for optimal actions..
		optimalActions = new double[numberOfStates];

		//..as the mid range of admissible actions at each state (assuming they are ordered)
		for (int stateIndex : nonAbsorbingStatesIndices) {
			double[] possibleActions = computeActions(states[stateIndex]);
			optimalActions[stateIndex]=(possibleActions[0]+possibleActions[possibleActions.length-1])/2;
		}

		/*
		 * We have to compute them to get the first update of the value functions as the solution
		 * of the linear system as described in the script. The computation depends on the specific
		 * problem, so the methods are abstract. 
		 */
		double[] knownTerm = computeConstantTerm(optimalActions);
		double[][] systemMatrix = computeSystemMatrix(optimalActions);
		
		//the solution of the linear system is computed here..
		double[] solutionLinearSystem = UsefulMethodsArrays.solveLinearSystem(systemMatrix, knownTerm);

		//..and assigned to the non-absorbing states here
		for (int solutionVectorIndex = 0; solutionVectorIndex< solutionLinearSystem.length; solutionVectorIndex++) {
			valueFunctions[nonAbsorbingStatesIndices[solutionVectorIndex]]
					= solutionLinearSystem[solutionVectorIndex];
		}
		
		//you can check that they are computed depending on the value of valueFunctions
		optimalActions = computeOptimalActions();
		
		//we have to know it in the while loop for checking the difference
		oldValueFunctions = valueFunctions.clone();
		
		//so we know it is bigger than requiredPrecision and that the loop starts
		double differenceBetweenPastAndOldValueFunctions = Double.MAX_VALUE;
		
		while (differenceBetweenPastAndOldValueFunctions >= requiredPrecision) {

			/*
			 * We have to compute them to get the updated value functions as the solution of the linear
			 * system as described in the script. The computation depends on the specific problem, so the methods
			 * are abstract. 
			 */
			knownTerm = computeConstantTerm(optimalActions);
			systemMatrix = computeSystemMatrix(optimalActions);	
			
			//the solution of the linear system is computed here..
			solutionLinearSystem = UsefulMethodsArrays.solveLinearSystem(systemMatrix, knownTerm);

			//..and assigned to the non-absorbing states here
			for (int solutionVectorIndex = 0; solutionVectorIndex< solutionLinearSystem.length; solutionVectorIndex++) {
				valueFunctions[nonAbsorbingStatesIndices[solutionVectorIndex]]
						= solutionLinearSystem[solutionVectorIndex];
			}	

			//we store it in a new record (i.e., we "add" a new record and we store it there) of updatedValueFunctions
			updatedValueFunctions.add(valueFunctions.clone()); 

			/*
			 * We check the maximum absolute difference between the new and the old value function: if this is smaller than
			 * requiredPrecision, the loop stops
			 */
			differenceBetweenPastAndOldValueFunctions = UsefulMethodsArrays.getMaxDifference(valueFunctions, oldValueFunctions);
			optimalActions = computeOptimalActions();
			
			//update of the old value functions
			oldValueFunctions = valueFunctions.clone();
		}

		//the loop is now terminated: we get the optimal actions
		optimalActions = computeOptimalActions();
	}


	// This method gets called at every iteration of the while loop above: it computes the updated optimal actions
	private double[] computeOptimalActions() {

		//one (updated) optimal action for every state
		double[] optimalActions = new double[numberOfStates];

		//actions at non-absorbing states
		for (int stateIndex : nonAbsorbingStatesIndices) {

			//the possible actions for the state
			double[] actions = computeActions(states[stateIndex]);

			//the returns for those actions
			double[] actionReturns = computeExpectedReturnsForStateAndActions(states[stateIndex], actions);

			//the index of the optimal action
			int indexOfOptimalAction = UsefulMethodsArrays.getMaximizingIndex(actionReturns);
			optimalActions[stateIndex] = actions[indexOfOptimalAction];
		}
		
		//actions at absorbing states: just NaN
		for (int stateIndex : absorbingStatesIndices) {
			optimalActions[stateIndex]=Double.NaN;
		}

		return optimalActions;
	}




	protected double[] getOldValueFunctions() {
		//needed to calculate the return of the actions in the derived classes
		return oldValueFunctions.clone();
	}

	/**
	 * It returns the discount factor 
	 * 
	 * @return the discount factor
	 */
	public double getDiscountFactor() {
		return discountFactor;
	}

	/**
	 * It returns a double array representing the value functions for every state
	 * 
	 * @return a double array representing the value functions for every state
	 * @throws Exception 
	 */
	public double[] getValueFunctions() throws Exception {
		if (valueFunctions == null) {
			//it gets called only once!
			generateValueFunctionsAndOptimalActions();
		}
		return valueFunctions;
	}

	/**
	 * It returns a double array representing the optimal actions providing the value functions for every state
	 * 
	 * @return a double array representing the value functions for every state
	 * @throws Exception 
	 */
	public double[] getOptimalActions() throws Exception {
		if (valueFunctions == null) {
			//it gets called only once!
			generateValueFunctionsAndOptimalActions();
		}
		return optimalActions.clone();
	}

	/**
	 * It returns the list of arrays of doubles recording all the updated value functions
	 * 
	 * @return the list of arrays of doubles recording all the updated value functions
	 * @throws Exception 
	 */
	@SuppressWarnings("unchecked")
	public ArrayList<double[]> getUpdatedOptimalValues() throws Exception {
		if (valueFunctions == null) {
			generateValueFunctionsAndOptimalActions();
		}
		return (ArrayList<double[]>) updatedValueFunctions.clone();
	}

	/**
	 * It computes and returns an array of doubles which represents the actions that are allowed for the given state
	 * @returns an array of doubles which represents the actions that are allowed for the given state
	 */
	protected abstract double[] computeActions(double state);


	/**
	 * It computes and returns an array of doubles which represents the (expected) returns associated to every action for a given state
	 * @returns an array of doubles which represents the actions that are allowed for the given state
	 */
	protected abstract double[] computeExpectedReturnsForStateAndActions(double state, double[] actions);

	/**
	 * It computes and returns the matrix that appers in the linear system to be solved to get the
	 * update value function for each update of the optimal action 
	 */
	protected abstract double[][] computeSystemMatrix(double[] actions);

	/**
	 * It computes and returns the constant term that appers in the linear system to be solved to get the
	 * update value function for each update of the optimal action 
	 */
	protected abstract double[] computeConstantTerm(double[] actions);

}
