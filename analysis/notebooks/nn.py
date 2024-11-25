import tensorflow as tf
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential # type: ignore
from tensorflow.keras.layers import LSTM, Dense, Dropout, BatchNormalization # type: ignore
from tensorflow.keras.optimizers import Adam # type: ignore
from tensorflow.keras.callbacks import ReduceLROnPlateau # type: ignore


class LSTMModel:
    """
    A class to handle the creation, compilation, and training of an LSTM-based model.
    """

    def __init__(self, _input_shape, _num_classes, _learning_rate=0.0001, _batch_size=50, _epochs=1000):
        """
        Initializes the LSTMModel with specified hyperparameters.

        Parameters:
            input_shape (tuple): Shape of the input data (timesteps, features).
            num_classes (int): Number of output classes for classification.
            learning_rate (float): Initial learning rate for the optimizer. Default is 0.0001.
            batch_size (int): Batch size for training. Default is 50.
            epochs (int): Number of training epochs. Default is 1000.
        """
        self.input_shape = _input_shape
        self.num_classes = _num_classes
        self.learning_rate = _learning_rate
        self.batch_size = _batch_size
        self.epochs = _epochs
        self.model = None

    def build_model(self):
        """
        Builds the LSTM model with the desired architecture.

        Architecture:
            - LSTM layer with 128 units, followed by BatchNormalization and Dropout.
            - LSTM layer with 32 units, followed by BatchNormalization and Dropout.
            - Dense layer with 32 units and ReLU activation.
            - Output Dense layer with softmax activation for classification.
        """
        self.model = Sequential([
            LSTM(128, input_shape=self.input_shape, return_sequences=True),
            BatchNormalization(),
            Dropout(0.3),
            LSTM(64, return_sequences=False),
            BatchNormalization(),
            Dropout(0.3),
            Dense(64, activation='relu'),
            Dense(self.num_classes, activation='softmax')
        ])
        print("Model built successfully.")
        self.model.summary()

    def compile_model(self):
        """
        Compiles the LSTM model using Adam optimizer and categorical cross-entropy loss.

        Metrics:
            - Accuracy.
        """
        optimizer = Adam(learning_rate=self.learning_rate)
        self.model.compile(
            optimizer=optimizer,
            loss='categorical_crossentropy',
            metrics=['accuracy']
        )
        print("Model compiled successfully.")

    def train_model(self, X_train, y_train, X_val, y_val):
        """
        Trains the LSTM model using the provided training and validation datasets.

        Parameters:
            X_train (numpy.array): Training data.
            y_train (numpy.array): Training labels.
            X_val (numpy.array): Validation data.
            y_val (numpy.array): Validation labels.

        Returns:
            history: A History object containing the training process details.
        """
        reduce_lr = ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.5,
            patience=5,
            min_lr=1e-6
        )
        print("Starting training...")
        history = self.model.fit(
            X_train, y_train,
            validation_data=(X_val, y_val),
            epochs=self.epochs,
            batch_size=self.batch_size,
            verbose=1,
            callbacks=[reduce_lr]
        )
        print("Training completed.")
        return history

    def save_model(self, file_path):
        """
        Saves the trained LSTM model to a specified file path.

        Parameters:
            file_path (str): Path to save the model.
        """
        self.model.save(file_path)
        print(f"Model saved to {file_path}.")

    def load_model(self, file_path):
        """
        Loads an LSTM model from a specified file path.

        Parameters:
            file_path (str): Path to the saved model file.
        """
        self.model = tf.keras.models.load_model(file_path)
        print(f"Model loaded from {file_path}.")

    def summary(self):
        """
        Prints the summary of the LSTM model's architecture.
        """
        return self.model.summary()

    def evaluate_model(self, X_test, y_test):
        """
        Evaluates the LSTM model on the test dataset.

        Parameters:
            X_test (numpy.array): Test data.
            y_test (numpy.array): Test labels.

        Returns:
            tuple: A tuple containing test loss and test accuracy.
        """
        test_loss, test_accuracy = self.model.evaluate(X_test, y_test, verbose=1)
        print(f"Test Loss: {test_loss:.4f}, Test Accuracy: {test_accuracy:.4f}")
        return test_loss, test_accuracy

    def plot_training_history(self, history):
        """
        Plots the training and validation loss and accuracy over epochs.

        Parameters:
            history: A History object containing the details of the training process.
        """
        # Loss plot
        plt.plot(history.history['loss'], label='Training Loss')
        plt.plot(history.history['val_loss'], label='Validation Loss')
        plt.title('Training and Validation Loss')
        plt.xlabel('Epochs')
        plt.ylabel('Loss')
        plt.legend()
        plt.show()

        # Accuracy plot
        plt.plot(history.history['accuracy'], label='Training Accuracy')
        plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
        plt.title('Training and Validation Accuracy')
        plt.xlabel('Epochs')
        plt.ylabel('Accuracy')
        plt.legend()
        plt.show()

    def predict_new_data(self, new_data, encoder):
        """
        Predicts the class for new input data.

        Parameters:
            new_data (numpy.array): Input data for prediction.
            encoder (sklearn.preprocessing.OneHotEncoder): Encoder to decode the predicted labels.

        Returns:
            numpy.array: Predicted labels.
        """
        predictions = self.model.predict(new_data)
        predicted_label = encoder.inverse_transform(predictions)
        print(f"Prediction: {predicted_label}")
        return predicted_label

    def validate_sample(self, test_data, real_labels, encoder, sample_index):
        """
        Validates a specific sample from the test dataset by comparing the prediction with the actual label.

        Parameters:
            test_data (pd.DataFrame): Test data containing the 'jerk_equivalent' column.
            real_labels (list): List of actual labels.
            encoder (sklearn.preprocessing.OneHotEncoder): Encoder to decode the predicted labels.
            sample_index (int): Index of the sample to validate.

        Returns:
            None
        """
        test1 = test_data['jerk_equivalent'].values
        test1 = test1.reshape(1, -1, 1)

        predictions = self.model.predict(test1)
        predicted_label = encoder.inverse_transform(predictions)

        real_label = real_labels[sample_index]

        print(f"Actual Label: {real_label}")
        print(f"Predicted Label: {predicted_label[0]}")

        if real_label == predicted_label[0]:
            print("Correct Prediction 🎉")
        else:
            print("Incorrect Prediction ❌")

