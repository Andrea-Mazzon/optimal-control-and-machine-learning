package it.univr.controlledmarkovchains.handout2solution;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import it.univr.usefulmethodsarrays.UsefulMethodsArrays;

/**
 * The main goal of this class is to provide the solution of a stochastic control problem in the setting
 * of controlled Markov chains for discrete time and discrete space, under the hypothesis that the transition
 * probabilities from one state to the other (i.e., the probabilities defining "the environment") are not known.
 * At this level, a general temporal difference learning is applied. The procedure is repeated over multiple "episodes":
 * every episode ends when one of the absorbing states is reached.
 *  
 * @author Andrea Mazzon
 *
 */
public abstract class TemporalDifferenceLearning {

	//the final rewards for every state. They must be zero for non absorbing states.
	private double[] rewardsAtStates;

	//the discount factor gamma in the notes
	private double discountFactor;

	//the running rewards, as a matrix: runningRewards[i][j] is the running reward for the i-th state and for the j-th action
	private double[][] runningRewards;

	//this array will contain the value function for every state, that is, the maximized values
	private double[] valueFunctions;

	//the indices of the optimal actions for every state
	private int[] optimalActionsIndices;

	//the Q-values, as a matrix: currentQValue[i][j] is the current Q-value for the i-th state and for the j-th action. It will get updated.
	private double[][] currentQValue;

	private int numberOfStates;

	private int numberOfEpisodes;


	// The learning rate lambda that enters in the update rule of the Q-values
	private double learningRate;

	//used to generate the random numbers to determine which state to start with
	private Random randomNumbersGenerator = new Random();

	//it will be used to check if a state index corresponds to an absorbing state
	List<Integer> absorbingStatesIndicesAsList;

	/**
	 * It constructs an object to solve a stochastic control problem in the setting of controlled Markov chains
	 * for discrete time and discrete space, under the hypothesis that the transition probabilities from one state
	 * to the other (i.e., the probabilities defining "the environment") are not known.
	 * 
	 * @param rewardsAtStates, the final rewards for every state. They must be zero for non absorbing states.
	 * @param discountFactor, the discount factor gamma in the notes
	 * @param runningRewards, the running rewards, as a matrix: runningRewards[i][j] is the running rewards for the i-th
	 *            state and for the j-th action
	 * @param numberOfEpisodes, the number of loops from an initial state until an absorbing state 
	 * @param learningRate, the learning rate lambda that enters in the update of the Q-values
	 */
	public TemporalDifferenceLearning(double[] rewardsAtStates, int[] absorbingStatesIndices, double discountFactor, double[][] runningRewards, int numberOfEpisodes,
			double learningRate) {
		this.rewardsAtStates = rewardsAtStates;
		numberOfStates = rewardsAtStates.length;
		absorbingStatesIndicesAsList = Arrays.stream(absorbingStatesIndices).boxed().toList();
		this.discountFactor = discountFactor;
		this.runningRewards = runningRewards;
		this.numberOfEpisodes = numberOfEpisodes; 
		this.learningRate = learningRate;
	}

	/*
	 * This is a private method which is used to fill the currentQValue matrix, which represents the Q-value for every state and action.
	 * The value function and the index of the best action for the i-th state are then computed as max_{j}currentQValue[i,j] and
	 * argmax_{j}currentQValue[i,j], respectively.
	 */
	private void generateOptimalValueAndPolicy() {

		valueFunctions = new double[numberOfStates];
		optimalActionsIndices = new int[numberOfStates];

		int numberOfActions = getNumberOfActions();
		currentQValue = new double[numberOfStates][numberOfActions];

		/*
		 * We give the initial Q-values: for actions which are not permitted for a given state, we set the Q-value to minus Infinity.
		 * For actions which are permitted, we make the initial value of the Q-value equal to the final reward for the associated state,
		 * independently of the action
		 */
		for (int stateIndex = 0; stateIndex < numberOfStates; stateIndex ++) {
			if (absorbingStatesIndicesAsList.contains(stateIndex)) {
				for (int actionIndex = 0; actionIndex < numberOfActions; actionIndex ++) {
					currentQValue[stateIndex][actionIndex]=rewardsAtStates[stateIndex];
				}
			} else {
				//the index of the actions which are allowed for that state
				int[] possibleActionsIndices = computePossibleActionsIndices(stateIndex);

				//we make it a list because then it's easier to check if it contains the given action indices
				List<Integer> possibleActionsIndicesAsList = Arrays.stream(possibleActionsIndices).boxed().toList();

				//the column index is the action index
				for (int actionIndex = 0; actionIndex < numberOfActions; actionIndex ++) {
					currentQValue[stateIndex][actionIndex]=possibleActionsIndicesAsList.contains(actionIndex) ?
							0 : Double.NEGATIVE_INFINITY;
				}
			}
		}

		//now we go through the episodes

		//any episode starts from a randomly chosen state and terminates when hitting an absorbing state
		for (int episodeIndex = 0; episodeIndex < numberOfEpisodes; episodeIndex ++) {

			//we generate a possible state
			int temptativeStateIndex = randomNumbersGenerator.nextInt(numberOfStates);

			//if it is an absorbing state, we want to generate another one, and so on
			while (absorbingStatesIndicesAsList.contains(temptativeStateIndex)) {
				temptativeStateIndex = randomNumbersGenerator.nextInt(numberOfStates);
			}

			//finally, we get the state which is not absorbing
			int stateIndex = temptativeStateIndex;

			/*
			 * For off-policy methods, this is a dummy method. For on policy methods, this is chosen based on the stateIndex
			 * and it is the index of the first action that is followed: the other ones are determined at the end of the while loop.
			 */
			int candidateActionIndex = chooseCandidateActionIndex(stateIndex);

			while (true) {//it ends when we land in an absorbing state

				/*
				 * It is the index of "a" in the notes. For off-policy methods, this is chosen based on stateIndex, whereas for on-policy
				 * methods it will be equal to candidateActionIndex.
				 */
				int chosenActionIndex = chooseActionIndex(stateIndex, candidateActionIndex);

				/*
				 * The index of the new state (i.e., of x' in the notes), randomly picked in a way which depends on the action and on the state.
				 * The way is chosen depends on the specific problem.
				 */
				int newStateIndex = generateStateIndex(stateIndex, chosenActionIndex);

				if (absorbingStatesIndicesAsList.contains(newStateIndex)) {
					//if we land at an absorbing state, there is no possible action to be taken: the maximum is equal to the reward
					currentQValue[stateIndex][chosenActionIndex] = currentQValue[stateIndex][chosenActionIndex] +
							learningRate * (discountFactor*rewardsAtStates[newStateIndex]-currentQValue[stateIndex][chosenActionIndex]) ;
					break; //we exit the while loop
				}

				/*
				 * This method returns an array of doubles whose second element is the value u_p(x',a') to be given in the update formula
				 * Q(x,a) <- Q(x,a) + lambda * (f^a(x)+gamma*u_p(x',a')-Q(x,a)), whereas the first one the index of a'.
				 * For off-policy methods, only the value u_p(x',a') matters, because a' is not followed later.
				 * For on-policy methods, instead, also a' is important, because it will be the next action to be followed. 
				 */
				double[] newCandidateActionIndexAndValue = getCandidateActionIndexAndValue(newStateIndex);

				//it does not actually matter for off-policy methods. For on-policy methods it determines chosenActionIndex at the next step
				candidateActionIndex = (int) newCandidateActionIndexAndValue[0];//a'

				double newValue = newCandidateActionIndexAndValue[1];//enters in the update formula

				//update
				currentQValue[stateIndex][chosenActionIndex] = currentQValue[stateIndex][chosenActionIndex] +
						learningRate * (runningRewards[stateIndex][chosenActionIndex] + discountFactor*newValue-currentQValue[stateIndex][chosenActionIndex]);

				stateIndex = newStateIndex;

			}
		}

		//now we have run all the episodes, so we have our "final" currentQValue matrix. We then compute the value functions and the optimal actions
		for (int stateIndexAtTheEnd = 0; stateIndexAtTheEnd < numberOfStates; stateIndexAtTheEnd ++) {
			if (absorbingStatesIndicesAsList.contains(stateIndexAtTheEnd)) { 
				//no action is possible in the absorbing states
				valueFunctions[stateIndexAtTheEnd] = rewardsAtStates[stateIndexAtTheEnd];
				optimalActionsIndices[stateIndexAtTheEnd] = (int) Double.NaN;
			} else {
				valueFunctions[stateIndexAtTheEnd] = UsefulMethodsArrays.getMax(currentQValue[stateIndexAtTheEnd]);
				optimalActionsIndices[stateIndexAtTheEnd] = UsefulMethodsArrays.getMaximizingIndex(currentQValue[stateIndexAtTheEnd]);
			}
		}
	}


	/**
	 * It returns the most updated version of currentQValue
	 * @return the most updated version of currentQValue
	 */
	protected double[][] getCurrentQValue() {
		return currentQValue.clone();
	}

	/**
	 * It returns a double array representing the value functions for every state
	 * 
	 * @return a double array representing the value functions for every state
	 */
	public double[] getValueFunctions() {
		if (currentQValue == null) {
			generateOptimalValueAndPolicy();
		}
		return valueFunctions.clone();
	}

	/**
	 * It returns a double array representing the optimal actions providing the value functions for every state
	 * 
	 * @return a double array representing the value functions for every state
	 */
	public int[] getOptimalActionsIndices() {
		if (valueFunctions == null) {
			generateOptimalValueAndPolicy();
		}
		return optimalActionsIndices.clone();
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
	 * It computes and returns the index of the chosen action at the beginning of an episode
	 * 
	 * @param stateIndex, the index that has been chosen at the beginning of an episode
	 * @return  the index of the chosen action at the beginning of an episode
	 */
	protected abstract int chooseCandidateActionIndex(int stateIndex);

	/**
	 * It returns the index of the chosen action "a" at an iteration of an episode. For off-policy methods
	 * this is equal to candidateActionIndex
	 * @param stateIndex
	 * @param candidateActionIndex
	 * @return the index of the chosen action "a" at an iteration of an episode. For off-policy methods
	 * this is equal to candidateActionIndex
	 */
	protected abstract int chooseActionIndex(int stateIndex, int candidateActionIndex);


	/**
	 * It returns the value u_p(x',a') that must be put in the update formula
	 * Q(x,a) <- Q(x,a) + lambda * (f^a(x)+gamma*u_p(x',a')-Q(x,a)),
	 * and the index of a'
	 * 
	 * @param stateIndex, the index of x'
	 * @return
	 */
	protected abstract double[] getCandidateActionIndexAndValue(int stateIndex);


	/**
	 * It returns the total number of possible actions (indipendent from the state)
	 * @return the total number of possible actions (indipendent from the state)
	 */
	protected abstract int getNumberOfActions();

	/**
	 * It computes and returns an array of integers which represents the indices of the actions that are allowed for the given state
	 * @return an array of integers which represents the indices of the actions that are allowed for the given state
	 */
	protected abstract int[] computePossibleActionsIndices(int stateIndex);

	/**
	 * It (randomly) generates the index of the next state, based on the old state index and on the chosen action index
	 * 
	 * @param oldStateIndex
	 * @param actionIndex
	 * @return the index of the next state
	 */
	protected abstract int generateStateIndex(int oldStateIndex, int actionIndex);
}
