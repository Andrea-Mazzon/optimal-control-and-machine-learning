package it.univr.controlledmarkovchains.handout2solution;

import java.util.Random;

import it.univr.usefulmethodsarrays.UsefulMethodsArrays;

/**
 * The main goal of this class is to provide the solution of a stochastic control problem in the setting
 * of controlled Markov chains for discrete time and discrete space, under the hypothesis that the transition
 * probabilities from one state to the other (i.e., the probabilities defining "the environment") are not known.
 * In particular, the Sarsa method is applied. The procedure is repeated over multiple "episodes":
 * every episode ends when one of the absorbing states is reached.
 *  
 * @author Andrea Mazzon
 *
 */
public abstract class SarsaInheritance extends TemporalDifferenceLearning{

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
	 * to the other (i.e., the probabilities defining "the environment") are not known. The Sarsa algorithm is applied.
	 * 
	 * @param rewardsAtStates, the final rewards for every state. They must be zero for non absorbing states.
	 * @param discountFactor, the discount factor gamma in the notes
	 * @param runningRewards, the running rewards, as a matrix: runningRewards[i][j] is the running rewards for the i-th
	 *            state and for the j-th action
	 * @param numberOfEpisodes, the number of loops from an initial state until an absorbing state 
	 * @param learningRate, the learning rate lambda that enters in the update of the Q-values
	 * @param explorationProbability: an action a at a given state x is randomly chosen in the set of possible actions for x
	 *            with probability equal to explorationProbability, and is instead chosen as the maximizing action for the Q-value
	 *            in x with probability equal to 1 - explorationProbability
	 */
	public SarsaInheritance(double[] rewardsAtStates, int[] absorbingStatesIndices, double discountFactor,
			double[][] runningRewards, int numberOfEpisodes, double learningRate, double explorationProbability) {
		super(rewardsAtStates, absorbingStatesIndices, discountFactor, runningRewards, numberOfEpisodes, learningRate);

		//this is the only parameter specific of this class
		this.explorationProbability = explorationProbability;
	}


	//note that this method gets called at the beginning of every episode
	protected int chooseCandidateActionIndex(int stateIndex) {

		int chosencandidateActionIndex;

		if (generator.nextDouble()< explorationProbability){//exploration: randomly chosen action
			int[] possibleActionsIndices = computePossibleActionsIndices(stateIndex);
			chosencandidateActionIndex = possibleActionsIndices[generator.nextInt(possibleActionsIndices.length)];
		} else {//exploitation: one maximizing action                
			double[][] qValue = getCurrentQValue();
			chosencandidateActionIndex = UsefulMethodsArrays.getMaximizingIndex(qValue[stateIndex]);
		}
		return chosencandidateActionIndex;
	}


	/*
	 * Note that this method gets called to choose the new a': in this case is candidateActionIndex:
	 * it is the action computed in the method above if we are at the first iteration of an episode,
	 * and the action returned by the next method at the previous iteration otherwise
	 */
	protected int chooseActionIndex(int stateIndex, int candidateActionIndex) {
		return candidateActionIndex;//a=a'
	}


	protected double[] getCandidateActionIndexAndValue(int stateIndex) {

		double[][] currentQValue = getCurrentQValue();

		int chosenActionIndex; //this will be the action for the next iteration
		double valueUsedToUpdate; //this will update the current Q value

		if (generator.nextDouble()< explorationProbability){//exploration: randomly chosen action
			int[] possibleNewActionsIndices = computePossibleActionsIndices(stateIndex);
			chosenActionIndex = possibleNewActionsIndices[generator.nextInt(possibleNewActionsIndices.length)];
		} else {//exploitation: one maximizing action                           
			chosenActionIndex = UsefulMethodsArrays.getMaximizingIndex(currentQValue[stateIndex]);
		}
		valueUsedToUpdate = currentQValue[stateIndex][chosenActionIndex];

		return new double[] {chosenActionIndex, valueUsedToUpdate};
	}



}