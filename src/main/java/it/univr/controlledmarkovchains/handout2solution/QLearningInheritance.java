package it.univr.controlledmarkovchains.handout2solution;

import java.util.Random;

import it.univr.usefulmethodsarrays.UsefulMethodsArrays;

/**
 * The main goal of this class is to provide the solution of a stochastic control problem in the setting
 * of controlled Markov chains for discrete time and discrete space, under the hypothesis that the transition
 * probabilities from one state to the other (i.e., the probabilities defining "the environment") are not known.
 * In particular, the Q-learning method is applied. The procedure is repeated over multiple "episodes":
 * every episode ends when one of the absorbing states is reached.
 *  
 * @author Andrea Mazzon
 *
 */
public abstract class QLearningInheritance extends TemporalDifferenceLearning{

	/*
	 * A parameter in [0,1] that characterizes the exploration probability: an action a at a given state x is
	 * randomly chosen in the set of possible actions for x with probability equal to explorationProbability, and is instead
	 * chosen as the maximizing action for the Q-value in x with probability equal to 1 - explorationProbability
	 */
	private double explorationProbability;
	
	//used to generate the random numbers to determine exploration or exploitation
	private Random generator = new Random();
	
	/**
	 * It constructs an object to solve a stochastic control problem in the setting of controlled Markov chains
	 * for discrete time and discrete space, under the hypothesis that the transition probabilities from one state
	 * to the other (i.e., the probabilities defining "the environment") are not known. The Q-learning algorithm is applied.
	 * 
	 * @param rewardsAtStates, the final rewards for every state. They must be zero for non absorbing states.
	 * @param discountFactor, the discount factor gamma in the notes
	 * @param runningRewards, the running rewards, as a matrix: runningRewards[i][j] is the running rewards for the i-th
	 * 		  state and for the j-th action
	 * @param numberOfEpisodes, the number of loops from an initial state until an absorbing state 
	 * @param learningRate, the learning rate lambda that enters in the update of the Q-values
	 * @param explorationProbability: an action a at a given state x is randomly chosen in the set of possible actions for x
	 * 		  with probability equal to explorationProbability, and is instead chosen as the maximizing action for the Q-value in x
	 * 		  with probability equal to 1 - explorationProbability
	 */
	public QLearningInheritance(double[] rewardsAtStates, int[] absorbingStatesIndices, double discountFactor,
			double[][] runningRewards, int numberOfEpisodes, double learningRate, double explorationProbability) {
		super(rewardsAtStates, absorbingStatesIndices, discountFactor, runningRewards, numberOfEpisodes, learningRate);
		
		//this is the only parameter specific of this class
		this.explorationProbability = explorationProbability;
	}


	protected int chooseCandidateActionIndex(int stateIndex) {
		return 0;//dummy implementation: it does not matter
	}


	//note that this method gets called to choose the new a'
	protected int chooseActionIndex(int stateIndex, int candidateActionIndex) {
	
		int chosenActionIndex;
		
		if (generator.nextDouble()< explorationProbability){//exploration: randomly chosen action
			int[] possibleActionsIndices = computePossibleActionsIndices(stateIndex);
			chosenActionIndex = possibleActionsIndices[generator.nextInt(possibleActionsIndices.length)];
		} else {//exploitation: one maximizing action			
			double[][] currentQValue = getCurrentQValue();
			chosenActionIndex = UsefulMethodsArrays.getMaximizingIndex(currentQValue[stateIndex]);
		}
		return chosenActionIndex;
	}
	
	

	protected double[] getCandidateActionIndexAndValue(int stateIndex) {
		double[][] currentQValue = getCurrentQValue();
		
		int[] possibleNewActionsIndices = computePossibleActionsIndices(stateIndex);

		//we fill this array and then take its maximum
		double[]  valuesForPossibleActions = new double[possibleNewActionsIndices.length];

		for (int indexForNewStates = 0; indexForNewStates< valuesForPossibleActions.length; indexForNewStates ++ ) {
			valuesForPossibleActions[indexForNewStates] = currentQValue[stateIndex][possibleNewActionsIndices[indexForNewStates]];
		}

		//we only care about the maximum, not about the action determining that maximum
		double maximumForGivenStateIndex = UsefulMethodsArrays.getMax(valuesForPossibleActions);

		return new double[] {0, maximumForGivenStateIndex};
	}

}
