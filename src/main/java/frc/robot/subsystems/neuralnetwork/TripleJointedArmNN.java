package frc.robot.subsystems.neuralnetwork;

import org.deeplearning4j.datasets.iterator.impl.ListDataSetIterator;
import org.deeplearning4j.nn.conf.MultiLayerConfiguration;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.lossfunctions.LossFunctions;
import org.nd4j.linalg.dataset.api.iterator.DataSetIterator;
import org.nd4j.linalg.dataset.DataSet;
import org.deeplearning4j.optimize.listeners.ScoreIterationListener;
import org.nd4j.linalg.learning.config.Adam;

public class TripleJointedArmNN {
    /*
    NOTES FOR CHANGING VARIABLES:
    Epoch Num:
         - # of times dataset will be passed through the neural network
    Batch Size:
         - In each epoch, the training data is divided into multiple batches, each of size equal to the 
           batch size.
         - A smaller batch size means the network updates its weights more frequently, which can 
           lead to a faster learning process but can also result in more noise in the learning process
    Changing Neurons:
         - Be very careful when changing neurons to prevent overfitting, start small and scale up
    */
    private int firstLayerNeurons;
    private int secondLayerNeurons;
    private int batchSize; 
    private int numEpochs; 

    private MultiLayerNetwork model;

    // MAKE SURE TO TUNE THESE HYPER PARAMETERS
    public TripleJointedArmNN() {
        firstLayerNeurons = 64;
        secondLayerNeurons = firstLayerNeurons / 2;
        batchSize = 64; 
        numEpochs = 10; 
        initializeNetwork();
    }

    public TripleJointedArmNN(boolean tahaMode) {
        if (tahaMode){
            firstLayerNeurons = 100;
            secondLayerNeurons = firstLayerNeurons / 2;
            batchSize = 32; 
            numEpochs = 10;
            initializeNetwork();
        } else {
            firstLayerNeurons = 64;
            secondLayerNeurons = firstLayerNeurons / 2;
            batchSize = 64; 
            numEpochs = 10; 
            initializeNetwork();
        }
    }

    private void initializeNetwork() {
        int numInputs = 3;  
        int numOutputs = 3; 

        MultiLayerConfiguration conf = new NeuralNetConfiguration.Builder()
                .updater(new Adam(0.001)) 
                .list()
                .layer(new DenseLayer.Builder().nIn(numInputs).nOut(firstLayerNeurons)
                        .activation(Activation.RELU).build())
                .layer(new DenseLayer.Builder().nIn(firstLayerNeurons).nOut(secondLayerNeurons)
                        .activation(Activation.RELU).build())
                .layer(new OutputLayer.Builder(LossFunctions.LossFunction.MSE)
                        .activation(Activation.IDENTITY).nIn(secondLayerNeurons).nOut(numOutputs).build())
                .build();
        this.model = new MultiLayerNetwork(conf);
        this.model.init();
    }

    public void trainNetwork(double[][] trainingDataArray, double[][] trainingLabelsArray) {
        INDArray trainingData = Nd4j.create(trainingDataArray);
        INDArray trainingLabels = Nd4j.create(trainingLabelsArray);
        
        DataSet dataSet = new DataSet(trainingData, trainingLabels);

        DataSetIterator dataSetIterator = new ListDataSetIterator<DataSet>(dataSet.asList(), batchSize);

        model.setListeners(new ScoreIterationListener(1));

        for (int epoch = 0; epoch < numEpochs; epoch++) {
            model.fit(dataSetIterator);
        }
    }

    public double[] predictControlActions(double joint1EncVal, double joint2EncVal, double joint3EncVal) {
        double[] encoderValues = new double[] {joint1EncVal, joint2EncVal, joint3EncVal};
        INDArray input = Nd4j.create(encoderValues);
        INDArray output = model.output(input);
        return output.toDoubleVector();
    }
    


    /*
    // HOW WE WOULD USE THIS CLASS IN THE REAL WORLD: 
        public static void main(String[] args) {
            TripleJointedArmNN armNN = new TripleJointedArmNN();

            // Example training data (replace with real data)
            double[][] trainingDataArray = {
                {0.5, 0.6, 0.7}, // Encoder values for joint 1, 2, 3 at time t1
                {0.8, 0.9, 0.4}  // Encoder values for joint 1, 2, 3 at time t2
                // ... more data
            };

            // Example training labels (replace with real data)
            double[][] trainingLabelsArray = {
                {1.0, 0.0, 0.0}, // Desired control actions for joint 1, 2, 3 at time t1
                {0.0, 1.0, 0.0}  // Desired control actions for joint 1, 2, 3 at time t2
                // ... more labels
            };

            // Train the network
            armNN.trainNetwork(trainingDataArray, trainingLabelsArray);

            // Example prediction
            double[] predictedActions = armNN.predictControlActions(0.5, 0.6, 0.7);
            // Process the predicted actions as needed
        }
    */
}